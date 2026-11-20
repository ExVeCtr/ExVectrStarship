#!/usr/bin/env python3
"""Bench helper: talk to the vehicle over its USB serial directly, as a temporary ground station (node 0).

Run with the ground-station venv python (needs pyserial and the exvectr_gs package for the stream framing):
  /home/chris/Development/ExVectrStarship/ground-station/.venv/bin/python tools/bench_vehicle_usb.py \
      [--flash .pio/build/teensy40/firmware.hex] [--csv out.csv] STEP [STEP ...]

Steps (run in order):
  wait <s>                          watch for <s> seconds
  tc <type> <paramInt> [paramFloat] send one legacy telecommand (port 0x7F, 8 bytes <BBhf, subsystem 0)
  tc <Name> [paramFloat]            by name with its validation value: SystemReset, Arm, Disarm, SimulationMode,
                                    MissionBegin (paramFloat = start time s, default -20), ZeroEstimators
  sh <command>                      run a shell command (for example `exgs tc send ...`, which then goes over the
                                    radio link because the core's usb source is disabled)
  until <expr> <timeout_s>          watch until a Python expression over the latest state holds (mode, mm, phase,
                                    flags, zeroing, armed, pz, vz ...), fail after the timeout

Procedure: (1) `exgs sources disable usb` so the bench core releases the vehicle USB, (2) optionally flash a hex
(134 baud reboot request, teensy_loader_cli, one retry), (3) open the vehicle USB, send node-0 heartbeats at 1 Hz
(so the vehicle keeps seeing a ground station and does not latch radioConnectionLoss), decode VEHICLE_STATUS,
POSE and LOG_TEXT with the generated module of THIS checkout (lib/ExVectrPackets/python, so new fields such as
the ZEROING flag decode even when the running core is older), run the steps and print a status line every 0.5 s,
(4) always `exgs sources enable usb` at the end. Never touches the Router or any Espressif device.
"""
import argparse
import csv
import glob
import os
import struct
import subprocess
import sys
import threading
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "lib", "ExVectrPackets", "python"))
import exvectr_packets as xp  # noqa: E402  (this checkout's schema)
from exvectr_gs.wire.exvectrnet_1.framing import (  # noqa: E402
    HEARTBEAT_CHANNEL, NetFramer, build_data_frame, build_stream_frame, make_ctrl, TYPE_HEARTBEAT)

EXGS = "/home/chris/Development/ExVectrStarship/ground-station/.venv/bin/exgs"
LOADER = os.path.expanduser("~/.platformio/packages/tool-teensy/teensy_loader_cli")
TEENSY_GLOB = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_*"

TC_PORT, TC_QOS = 0x7F, 2
TC_BY_NAME = {  # name: (type, paramInt, default paramFloat)
    "SystemReset": (4, 0xC5, 0.0),
    "Arm": (5, 0xA5, 0.0),
    "Disarm": (5, 0, 0.0),
    "SimulationMode": (6, 0xA9, 0.0),
    "MissionBegin": (7, -20, 0.0),  # paramInt = initial mission time (s), < -10 runs Init and Startup
    "ZeroEstimators": (10, 0x2E, 10.0),
}
MODES = ["Startup", "SensorInit", "SensorCalib", "Ready", "Running", "Failure"]
MMODES = ["Idle", "Init", "Startup", "Running", "Finished"]


def exgs(*args):
    r = subprocess.run([EXGS, *args], text=True, capture_output=True, timeout=30)
    print("$ exgs", " ".join(args), "->", (r.stdout + r.stderr).strip()[:200], flush=True)
    return r.returncode == 0


def teensy_port(timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = sorted(glob.glob(TEENSY_GLOB))
        if p:
            return p[0]
        time.sleep(0.2)
    raise RuntimeError("Teensy USB serial did not appear")


def flash(hexfile):
    try:
        s = serial.Serial(teensy_port(3), 134)
        time.sleep(0.2)
        s.close()
    except Exception as e:  # already in the bootloader
        print("reboot request skipped:", e)
    time.sleep(1.0)
    for attempt in (1, 2):
        r = subprocess.run([LOADER, "--mcu=TEENSY40", "-v", hexfile], text=True, capture_output=True, timeout=60)
        out = (r.stdout + r.stderr).strip()
        print(out)
        if r.returncode == 0 and "error" not in out.lower():
            return
        print("flash attempt %d failed, retrying" % attempt)
        time.sleep(1.5)
    raise RuntimeError("could not flash " + hexfile)


class Vehicle:
    def __init__(self, port, csv_path=None):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.framer = NetFramer(0)
        self.lock = threading.Lock()
        self.st = {"mode": None, "mm": None, "phase": None, "flags": 0, "pos": [0.0] * 3, "vel": [0.0] * 3,
                   "q": [1.0, 0, 0, 0], "fail": 0, "vt": 0.0, "sens": 0}
        self.seq = 0
        self.t0 = time.time()
        self.stop = False
        self.csvw = None
        if csv_path:
            self.csvf = open(csv_path, "w", newline="")
            self.csvw = csv.writer(self.csvf)
            self.csvw.writerow(["t", "type", "a", "b", "c", "d", "e", "f", "g", "h"])
        self.rx = threading.Thread(target=self._rx, daemon=True)
        self.hb = threading.Thread(target=self._hb, daemon=True)
        self.rx.start()
        self.hb.start()

    def write(self, net_frame):
        with self.lock:
            self.ser.write(build_stream_frame(net_frame))

    def _hb(self):
        while not self.stop:
            H = xp.Heartbeat
            payload = H(role=xp.HeartbeatRole.GROUND_STATION, net_version=1, schema_hash=xp.SCHEMA_HASH,
                        uptime=round(time.time() - self.t0, 3), flags=0).encode()
            hdr = struct.pack("<BBBB", make_ctrl(TYPE_HEARTBEAT, H.QOS, H.LATEST), 3, 0xFF, 0)
            self.write(hdr + payload)
            time.sleep(1.0)

    def tc(self, type_, param_int, param_float):
        payload = struct.pack("<BBhf", type_, 0, param_int, param_float)
        self.write(build_data_frame(0, 1, TC_PORT, self.seq, payload, TC_QOS, False, 3))
        self.seq = (self.seq + 1) & 0xFF
        print("%7.2f  >> TC type=%d paramInt=0x%X paramFloat=%g" % (self.now(), type_, param_int & 0xFFFF,
                                                                      param_float), flush=True)

    def now(self):
        return time.time() - self.t0

    def _rx(self):
        while not self.stop:
            try:
                data = self.ser.read(4096)
            except serial.SerialException as e:  # transient USB hiccup: keep the session alive
                print("%7.2f  serial read error: %s" % (self.now(), e), flush=True)
                time.sleep(0.1)
                continue
            if not data:
                continue
            for f in self.framer.feed(data, 0):
                if getattr(f, "channel", None) is None or f.channel == HEARTBEAT_CHANNEL:
                    continue
                cls = xp.PACKETS_BY_PORT.get(f.channel)
                if cls is None:
                    continue
                try:
                    p = cls.decode(f.payload)
                except Exception:
                    continue
                t = self.now()
                if cls is xp.VehicleStatus:
                    self.st.update(mode=p.mode, mm=p.mission_mode, phase=p.mission_phase, flags=p.flags,
                                   fail=p.failure_bits, vt=p.t_ns * 1e-9, sens=p.sensor_state)
                    if self.csvw:
                        self.csvw.writerow([round(t, 3), "VS", p.mode, p.mission_mode, p.mission_phase, p.flags,
                                            p.failure_bits, round(p.t_ns * 1e-9, 3)])
                elif cls is xp.Pose:
                    self.st.update(pos=list(p.pos), vel=list(p.vel), q=list(p.q))
                    if self.csvw:
                        self.csvw.writerow([round(t, 3), "POSE", *[round(v, 4) for v in p.pos],
                                            *[round(v, 4) for v in p.vel], *[round(v, 4) for v in p.q[:2]]])
                elif cls is xp.LogText:
                    txt = p.text.decode(errors="replace") if isinstance(p.text, bytes) else str(p.text)
                    print("%7.2f  LOG %s" % (t, txt.strip()), flush=True)

    def env(self):
        s = self.st
        return {"mode": s["mode"], "mm": s["mm"], "phase": s["phase"], "flags": s["flags"],
                "zeroing": bool(s["flags"] & xp.VehicleStatus.FLAGS_ZEROING),
                "armed": bool(s["flags"] & xp.VehicleStatus.FLAGS_ARMED),
                "sim": bool(s["flags"] & xp.VehicleStatus.FLAGS_SIMULATION),
                "px": s["pos"][0], "py": s["pos"][1], "pz": s["pos"][2], "vz": s["vel"][2], "q": s["q"],
                "READY": 3, "RUNNING": 4, "FAILURE": 5, "STARTUP": 0}

    def line(self):
        s = self.st
        e = self.env()
        mode = MODES[s["mode"]] if s["mode"] is not None and s["mode"] < len(MODES) else s["mode"]
        mm = MMODES[s["mm"]] if s["mm"] is not None and s["mm"] < len(MMODES) else s["mm"]
        print("%7.2f  %-8s %-8s ph=%s Z=%d A=%d S=%d fail=0x%X sens=0x%03X pos=[%7.3f %7.3f %8.3f] vel=[%6.3f %6.3f %6.3f] "
              "q=[%.4f %.4f %.4f %.4f]" % (self.now(), mode, mm, s["phase"], e["zeroing"], e["armed"], e["sim"],
                                           s["fail"], s["sens"], *s["pos"], *s["vel"], *s["q"]), flush=True)

    def watch(self, seconds, until=None):
        end = time.time() + seconds
        nxt = 0
        while time.time() < end:
            if time.time() >= nxt:
                self.line()
                nxt = time.time() + 0.5
            if until is not None and eval(until, {}, self.env()):
                self.line()
                return True
            time.sleep(0.02)
        return until is None

    def close(self):
        self.stop = True
        time.sleep(0.2)
        self.ser.close()
        if self.csvw:
            self.csvf.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flash", help="hex file to flash first")
    ap.add_argument("--csv", help="write every VEHICLE_STATUS and POSE to this CSV")
    ap.add_argument("steps", nargs="*")
    a = ap.parse_args()
    ok = True
    if not exgs("sources", "disable", "usb"):
        print("could not disable the usb source")
        return 2
    try:
        time.sleep(1.0)
        if a.flash:
            flash(a.flash)
            time.sleep(2.0)
        v = Vehicle(teensy_port(), a.csv)
        try:
            for step in a.steps:
                w = step.split()
                if w[0] == "wait":
                    v.watch(float(w[1]))
                elif w[0] == "until":
                    if not v.watch(float(w[-1]), " ".join(w[1:-1])):
                        print("TIMEOUT waiting for", " ".join(w[1:-1]))
                        ok = False
                        break
                elif w[0] == "sh":  # run a shell command (for example an exgs tc send over the radio link)
                    cmd = step[3:]
                    r = subprocess.run(cmd, shell=True, text=True, capture_output=True, timeout=60)
                    print("%7.2f  $ %s -> %s" % (v.now(), cmd, (r.stdout + r.stderr).strip()[:200]), flush=True)
                elif w[0] == "tc":
                    if w[1] in TC_BY_NAME:
                        t, pi, pf = TC_BY_NAME[w[1]]
                        if len(w) > 2:
                            if w[1] == "MissionBegin":
                                pi = int(w[2])
                            else:
                                pf = float(w[2])
                        v.tc(t, pi, pf)
                    else:
                        v.tc(int(w[1], 0), int(w[2], 0), float(w[3]) if len(w) > 3 else 0.0)
                else:
                    raise ValueError("unknown step " + step)
        finally:
            v.close()
    finally:
        exgs("sources", "enable", "usb")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
