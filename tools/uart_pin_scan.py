#!/usr/bin/env python3
"""Find the Teensy 4.0 pins wired to the ExVectrLink ship radio node (ESP8285, node 3).

Run with the ground-station venv python (needs pyserial):
  /home/chris/Development/ExVectrStarship/ground-station/.venv/bin/python tools/uart_pin_scan.py [--no-restore]

Steps: (1) disable the ground-station core's vehicle USB source (`exgs sources disable usb`), (2) flash the scan
firmware (env teensy40_pinscan, tools/pinscan/pinscan.cpp), (3) phase 1 LISTEN on every UART RX pin (inputs only)
plus an edge-activity scan of all pads, then phase 2 TRANSMIT node-1 heartbeats on the candidate UART TX pins
one at a time (never servo/ESC pins) while polling node 3's NetworkStatus through the Router (age[1] ~ 0 means the
ship node just heard node 1), (4) restore: rebuild and flash the flight firmware of the branch that was checked
out and re-enable the usb source. Never touches the Router port or the Espressif device.
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys
import threading
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GS_BIN = "/home/chris/Development/ExVectrStarship/ground-station/.venv/bin"
EXGS = os.path.join(GS_BIN, "exgs")
PIO = os.path.expanduser("~/.platformio/penv/bin/pio")
LOADER = os.path.expanduser("~/.platformio/packages/tool-teensy/teensy_loader_cli")
TEENSY_GLOB = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_*"
SERVO_PINS = {0, 1, 7, 8, 22, 23, 24, 25, 28, 29}
UART_TX = {"Serial1": 1, "Serial2": 8, "Serial3": 14, "Serial4": 17, "Serial5": 20, "Serial6": 24, "Serial7": 29}
UART_RX = {"Serial1": 0, "Serial2": 7, "Serial3": 15, "Serial4": 16, "Serial5": 21, "Serial6": 25, "Serial7": 28}


def run(cmd, **kw):
    print("$", " ".join(cmd), flush=True)
    return subprocess.run(cmd, text=True, capture_output=True, **kw)


def exgs(*args, check=True):
    r = run([EXGS, *args], timeout=30)
    out = (r.stdout + r.stderr).strip()
    if out:
        print(out)
    if check and r.returncode != 0:
        raise RuntimeError("exgs %s failed: %s" % (" ".join(args), out))
    return r


def teensy_port(timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = sorted(glob.glob(TEENSY_GLOB))
        if p:
            return p[0]
        time.sleep(0.2)
    raise RuntimeError("Teensy USB serial did not appear")


def flash(hexfile):
    # Reboot request: open the Teensy serial port at 134 baud.
    try:
        port = teensy_port(3)
        s = serial.Serial(port, 134)
        time.sleep(0.2)
        s.close()
    except Exception as e:  # already in bootloader or no port
        print("reboot request skipped:", e)
    time.sleep(1.0)
    for attempt in (1, 2, 3):
        r = run([LOADER, "--mcu=TEENSY40", "-v", hexfile], timeout=60)
        print((r.stdout + r.stderr).strip())
        if r.returncode == 0:
            return
        print("flash attempt %d failed (%s), retrying" % (attempt, "error writing" if "error writing" in r.stderr + r.stdout else "?"))
        time.sleep(1.5)
    raise RuntimeError("could not flash " + hexfile)


class Scan:
    def __init__(self):
        self.ser = serial.Serial(teensy_port(), 115200, timeout=0.2)

    def cmd(self, text, done, timeout):
        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode())
        lines, t0 = [], time.time()
        while time.time() - t0 < timeout:
            ln = self.ser.readline().decode(errors="replace").strip()
            if ln:
                lines.append((time.time(), ln))
                print("  teensy:", ln, flush=True)
                if done in ln:
                    break
        return lines


class NodeWatch(threading.Thread):
    """Streams NetworkStatus of node 3 (via the Router link source) from the core."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples = []  # (host time, present[1], age[1])
        self.proc = None

    def run(self):
        self.proc = subprocess.Popen([EXGS, "watch", "--types", "NetworkStatus", "--json"], stdout=subprocess.PIPE, text=True)
        for ln in self.proc.stdout:
            try:
                d = json.loads(ln)
                f = d["f"]
            except Exception:
                continue
            if f.get("node") == 3:
                age = f["age"][1]
                self.samples.append((time.time(), f["present"][1], age))

    def stop(self):
        if self.proc:
            self.proc.terminate()


def parse_rx(lines):
    rows = []
    for _, ln in lines:
        m = re.match(r"RXPIN baud=(\d+) pin=(\d+) uart=(\w+) bytes=(\d+) zeros=(\d+) frames=(\d+) hb=(\d+) "
                     r"hb_src_mask=0x([0-9a-f]+) src_mask=0x([0-9a-f]+) bad_crc=(\d+) bad_cobs=(\d+) first=(\w*)", ln)
        if m:
            g = m.groups()
            rows.append(dict(baud=int(g[0]), pin=int(g[1]), uart=g[2], bytes=int(g[3]), zeros=int(g[4]), frames=int(g[5]),
                             hb=int(g[6]), hbmask=int(g[7], 16), srcmask=int(g[8], 16), badcrc=int(g[9]), badcobs=int(g[10]),
                             first=g[11]))
    return rows


def print_rx_table(rows):
    print("\n  baud    pin uart     bytes zeros frames hb  srcmask badcrc badcobs first bytes")
    for r in rows:
        print("  %-7d %-3d %-8s %-5d %-5d %-6d %-3d 0x%-5x %-6d %-7d %s" % (
            r["baud"], r["pin"], r["uart"], r["bytes"], r["zeros"], r["frames"], r["hb"], r["srcmask"], r["badcrc"],
            r["badcobs"], r["first"][:32]))


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def cobs_decode(b):
    out, i = bytearray(), 0
    while i < len(b):
        code = b[i]
        i += 1
        if code == 0 or i + code - 1 > len(b):
            return None
        out += b[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(b):
            out.append(0)
    return bytes(out)


def decode_capture(pin, cpu_hz, start_level, deltas):
    """Software UART decode of a level-change capture; returns a dict (baud estimate, bytes, stream frames)."""
    if len(deltas) < 10:
        return dict(pin=pin, edges=len(deltas))
    widths = sorted(b - a for a, b in zip(deltas, deltas[1:]) if b > a)
    bit_cyc = widths[max(1, len(widths) // 20)]  # ~5th percentile = about one bit
    est = cpu_hz / bit_cyc
    std = [74880, 115200, 230400, 250000, 460800, 500000, 921600, 1000000, 9600, 19200, 38400, 57600]
    baud = min(std, key=lambda x: abs(x - est))
    bit = cpu_hz / baud
    # level timeline: level after edge i
    lv = [start_level]
    for _ in deltas:
        lv.append(1 - lv[-1])

    def level_at(t):  # binary search
        lo, hi = 0, len(deltas)
        while lo < hi:
            m = (lo + hi) // 2
            if deltas[m] <= t:
                lo = m + 1
            else:
                hi = m
        return lv[lo]

    data = bytearray()
    t, end = 0.0, deltas[-1]
    # first falling edge
    idx = next((i for i, _ in enumerate(deltas) if lv[i] == 1), None)
    if idx is None:
        return dict(pin=pin, edges=len(deltas), est_baud=est)
    t = deltas[idx]
    framing_err = 0
    while t + 10 * bit < end:
        if level_at(t + 0.5 * bit) != 0:  # not in a start bit: advance to next falling edge
            nxt = next((deltas[i] for i in range(len(deltas)) if deltas[i] > t and lv[i] == 1), None)
            if nxt is None:
                break
            t = nxt
            continue
        v = 0
        for k in range(8):
            v |= level_at(t + (1.5 + k) * bit) << k
        if level_at(t + 9.5 * bit) != 1:
            framing_err += 1
        data.append(v)
        t += 9.7 * bit  # resync on the next falling edge after the stop bit
        nxt = next((deltas[i] for i in range(len(deltas)) if deltas[i] >= t - 0.3 * bit and lv[i] == 1), None)
        if nxt is None:
            break
        t = nxt
    frames = []
    for chunk in bytes(data).split(b"\x00"):
        if not chunk:
            continue
        dec = cobs_decode(chunk)
        if dec and 7 <= len(dec) <= 243 and crc16(dec[:-2]) == dec[-2] | dec[-1] << 8 and dec[0] == 1:
            frames.append(dict(ctrl=dec[1], src=dec[4], type=(dec[1] >> 4) & 3,
                               port=dec[5] if len(dec) > 6 and ((dec[1] >> 4) & 3) == 0 else None))
    return dict(pin=pin, edges=len(deltas), est_baud=round(est), baud=baud, bytes=len(data), framing_err=framing_err,
                first=bytes(data[:24]).hex(), frames=len(frames), srcs=sorted({f["src"] for f in frames}),
                heartbeats=sum(1 for f in frames if f["type"] == 1), ports=sorted({f["port"] for f in frames if f["port"] is not None}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="targeted re-check: listen 460800 only, 1 s edges, TX on pin 17 and bit-bang pad 31 only")
    ap.add_argument("--no-restore", action="store_true", help="leave the scan firmware on the Teensy")
    ap.add_argument("--listen-ms", type=int, default=4000)
    ap.add_argument("--tx-ms", type=int, default=3000)
    ap.add_argument("--gap-s", type=float, default=2.5)
    args = ap.parse_args()

    branch = subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
    print("flight-software branch to restore:", branch)
    report = {"branch": branch}

    # (1) free the Teensy port from the core
    r = exgs("sources", "disable", "usb", check=False)
    if r.returncode != 0 or "not implemented" in (r.stdout + r.stderr).lower():
        print("\nCORE CANNOT DISABLE THE USB SOURCE (older core code?). Stop the ground station, then wait; aborting.")
        return 2
    time.sleep(1.0)
    restored = False
    try:
        # (2) build + flash the scan firmware
        r = run([PIO, "run", "-e", "teensy40_pinscan"], cwd=ROOT, timeout=300)
        if r.returncode:
            print(r.stdout[-2000:], r.stderr[-2000:])
            raise RuntimeError("scan firmware build failed")
        flash(os.path.join(ROOT, ".pio/build/teensy40_pinscan/firmware.hex"))
        time.sleep(2.0)
        sc = Scan()
        sc.cmd("V", "PINSCAN", 3)

        nw = NodeWatch()
        nw.start()

        # (3a) phase 1: listen
        print("\n== Phase 1: LISTEN (inputs only) ==")
        found = None
        all_rows = {}
        for baud in ((460800,) if args.quick else (460800, 115200, 74880)):
            lines = sc.cmd("L %d %d" % (baud, args.listen_ms), "LISTEN done", args.listen_ms / 1000 + 5)
            rows = parse_rx(lines)
            all_rows[baud] = rows
            print_rx_table(rows)
            hit = [r for r in rows if r["hb"] > 0 and r["hbmask"] & (1 << 3)]
            if hit and baud == 460800:
                found = hit[0]
                break
            if hit:
                print("  node 3 heartbeats decode at %d baud on pin %d (unexpected baud)" % (baud, hit[0]["pin"]))
                found = hit[0]
                break
        report["listen"] = all_rows

        print("\n== Edge activity scan of all pads (inputs, no pull) ==")
        ems = 1000 if args.quick else 500
        lines = sc.cmd("E %d" % ems, "EDGE done", 34 * (ems / 1000 + 0.1) + 5)
        edges = {}
        for _, ln in lines:
            m = re.match(r"EDGE pin=(\d+) edges=(\d+) high_pct=(\d+)", ln)
            if m:
                edges[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))
        active = {p: e for p, e in edges.items() if e[0] > 50}
        print("  pads with activity (>50 edges / 0.5 s):", active or "none")
        report["edges"] = edges

        print("\n== Capture and software-decode of the active pads ==")
        report["capture"] = {}
        for pin in sorted(active):
            lines = sc.cmd("C %d 1500" % pin, "CAPTURE done", 8)
            cpu, lvl, deltas = 600000000, 1, []
            for _, ln in lines:
                m = re.match(r"CAPTURE pin=\d+ cpu_hz=(\d+) start_level=(\d+)", ln)
                if m:
                    cpu, lvl = int(m.group(1)), int(m.group(2))
                if ln.startswith("CAP "):
                    deltas += [int(x) for x in ln[4:].split()]
            dec = decode_capture(pin, cpu, lvl, deltas)
            report["capture"][pin] = dec
            print("  pad %d: %s" % (pin, dec))

        # (3b) phase 2: transmit
        print("\n== Phase 2: TRANSMIT node-1 heartbeats on candidate UART TX pins ==")
        cands = [p for p in sorted(UART_TX.values()) if p not in SERVO_PINS and p != 20]
        if args.quick:
            cands = [17]
        print("  candidates (servo/ESC pins %s are never driven): %s" % (sorted(SERVO_PINS), cands))
        results = {}
        if not nw.samples:
            print("  WARNING: no NetworkStatus from node 3 seen on the core; phase 2 would be blind")
        for pin in cands:
            t_start = time.time()
            sc.cmd("T %d %d" % (pin, args.tx_ms), "stop", args.tx_ms / 1000 + 4)
            t_stop = time.time()
            time.sleep(args.gap_s)
            # reports from node 3 are 1 Hz; allow ~1 s air latency
            win = [s for s in nw.samples if t_start + 0.3 <= s[0] <= t_stop + 1.3]
            ages = [s[2] for s in win if s[2] is not None]
            heard = [a for a in ages if a < 0.6]
            results[pin] = dict(reports=len(win), min_age=min(ages) if ages else None, heard=len(heard))
            print("  pin %d: %d node-3 reports, min age[1]=%s, reports with age[1]<0.6 s: %d  -> %s" % (
                pin, len(win), results[pin]["min_age"], len(heard), "NODE 3 HEARD NODE 1" if heard else "no"))
        report["tx"] = results

        # Phase 2b: the ship node's TX was found on a pad that is not a UART RX pin, so its RX may be on a pad without
        # a UART TX too. Candidates: pads that idle high with no activity and are no known board signal. Driven
        # open-drain only (pulled low or released, never driven high) by bit-banging.
        KNOWN_BOARD = {3, 5, 6, 11, 12, 13, 15, 18, 19, 20}
        uart_rx = set(UART_RX.values())
        nonuart_found = [p for p, d in report["capture"].items() if d.get("srcs") and 3 in d["srcs"] and p not in uart_rx] or ([31] if args.quick else [])
        bb = {}
        if nonuart_found:
            idle = [31] if args.quick else [p for p, (e, hp) in edges.items() if e == 0 and hp == 100 and p not in KNOWN_BOARD and p not in SERVO_PINS
                    and p not in UART_TX.values() and p not in uart_rx]
            print("\n  node 3 TX decoded on non-UART-RX pad(s) %s. Bit-bang (open-drain) candidates: %s" % (nonuart_found, idle))
            for pin in [p for p in idle if p == 31]:
                t_start = time.time()
                sc.cmd("B %d %d" % (pin, args.tx_ms), "BB pin %d stop" % pin, args.tx_ms / 1000 + 4)
                t_stop = time.time()
                time.sleep(args.gap_s)
                win = [s for s in nw.samples if t_start + 0.3 <= s[0] <= t_stop + 1.3]
                ages = [s[2] for s in win if s[2] is not None]
                heard = [a for a in ages if a < 0.6]
                bb[pin] = dict(reports=len(win), min_age=min(ages) if ages else None, heard=len(heard))
                print("  pad %d (bit-bang): %d reports, min age[1]=%s, with age<0.6: %d -> %s" % (
                    pin, len(win), bb[pin]["min_age"], len(heard), "NODE 3 HEARD NODE 1" if heard else "no"))
        report["bitbang"] = bb
        nw.stop()

        # (3c) conclusion
        print("\n== Conclusion ==")
        txhit = [p for p, r in results.items() if r["heard"]]
        rxs = None
        ncap = [p for p, d in report["capture"].items() if d.get("srcs") and 3 in d["srcs"]]
        if ncap:
            rxs = "ship node TX -> Teensy pad %d (node 3 stream decoded by software UART at %s baud; %s)" % (
                ncap[0], report["capture"][ncap[0]]["baud"],
                "a hardware UART RX pin" if ncap[0] in uart_rx else "NOT a hardware UART RX pin on the Teensy 4.0")
        elif found:
            rxs = "ship node TX -> Teensy pin %d (%s RX)" % (found["pin"], found["uart"])
        elif active:
            rxs = "no UART RX pin decoded frames, but activity on pads %s (see edge table; possibly a TX pin or wrong baud)" % sorted(active)
        else:
            rxs = "no ship node TX found on any Teensy UART RX pin or other pad"
        bbhit = [p for p, r in bb.items() if r["heard"]]
        if bbhit:
            txhit = bbhit
        txs = ("Teensy pin %d (%s TX) -> ship node RX" % (txhit[0], ([k for k, v in UART_TX.items() if v == txhit[0]] or ["bit-bang, no UART"])[0])
               if txhit else "no UART TX pin made the ship node hear node 1 (candidates %s)" % cands)
        print("  " + rxs)
        print("  " + txs)
        report["conclusion"] = [rxs, txs]
        print("\nJSON:", json.dumps(report, default=str))
    finally:
        if not args.no_restore:
            print("\n== Restore ==")
            try:
                r = run([PIO, "run"], cwd=ROOT, timeout=600)
                print((r.stdout + r.stderr)[-600:])
                if r.returncode:
                    raise RuntimeError("flight firmware build failed")
                flash(os.path.join(ROOT, ".pio/build/teensy40/firmware.hex"))
                restored = True
            except Exception as e:
                print("RESTORE FAILED:", e)
            time.sleep(2.0)
            exgs("sources", "enable", "usb", check=False)
            time.sleep(3.0)
            exgs("sources", "list", check=False)
            print("restored flight firmware from branch %s: %s" % (branch, restored))
    return 0


if __name__ == "__main__":
    sys.exit(main())
