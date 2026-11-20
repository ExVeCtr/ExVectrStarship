// UART pin scan bench firmware (NOT flight firmware, env teensy40_pinscan, links no flight code).
//
// Finds which two Teensy 4.0 pins the ExVectrLink ship radio node (ESP8285, node 3) is wired to.
// Driven by the host script tools/uart_pin_scan.py over USB serial text (one command per line):
//   V                 version / self-test
//   L <baud> <ms>     LISTEN on the RX pins of Serial1..Serial7 (inputs only), print per-pin statistics
//   E <ms>            EDGE scan: every pad 0..33 as plain high-impedance input, count level changes for <ms> each
//                     (finds activity on pads that are not UART RX pins, e.g. a crossed TX/RX wiring)
//   C <pin> <ms>      CAPTURE: record up to 6000 level-change timestamps (CPU cycles) on <pin> (input only), print them
//   B <pin> <ms>      BIT-BANG node-1 HEARTBEATs (460800 8N1, 1 per 100 ms) on a non-UART pad, OPEN-DRAIN style: the pad
//                     is only ever pulled low or released (high-Z), never driven high. Only pad 31 is supported.
//   T <pin> <ms>      TRANSMIT node-1 HEARTBEAT stream frames (1 per 100 ms, 460800 8N1) on UART TX pin <pin>
//   X <pin>           exclude <pin> from transmitting (the ship node's TX wire, found in the LISTEN phase)
//
// Safety:
// - The Teensy 4 core has no RX-only UART mode: HardwareSerial::begin() muxes the TX pin to the UART as well
//   (idle level high). In LISTEN every UART is therefore begun and its TX pad is switched back to a plain
//   high-impedance GPIO input (pinMode INPUT) immediately afterwards, so the TX pads are driven for a few
//   microseconds at most (idle high) and never carry data. Everything else is left untouched.
// - TRANSMIT refuses the servo/ESC pins (0 1 7 8 22 23 24 25 28 29) and excluded pins.
// - No servo, motor or flight library is initialised.
#include <Arduino.h>
#include <string.h>

namespace {

struct Uart {
    HardwareSerial* port;
    const char* name;
    uint8_t rxPin;
    uint8_t txPin;
};

Uart uarts[] = {
    {&Serial1, "Serial1", 0, 1},   {&Serial2, "Serial2", 7, 8},   {&Serial3, "Serial3", 15, 14},
    {&Serial4, "Serial4", 16, 17}, {&Serial5, "Serial5", 21, 20}, {&Serial6, "Serial6", 25, 24},
    {&Serial7, "Serial7", 28, 29},
};
constexpr size_t kUarts = sizeof(uarts) / sizeof(uarts[0]);

bool isServoPin(int pin) {
    switch (pin) {
        case 0: case 1: case 7: case 8: case 22: case 23: case 24: case 25: case 28: case 29: return true;
        default: return false;
    }
}

uint8_t excluded[8];
size_t nExcluded = 0;
bool isExcluded(int pin) {
    for (size_t i = 0; i < nExcluded; i++) if (excluded[i] == pin) return true;
    return false;
}

// ---- stream frame codec (workspace docs/network-design.md 4.2) -----------------------------------------

uint16_t crc16(const uint8_t* d, size_t n) {  // CRC-16/CCITT-FALSE
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}

size_t cobsEncode(const uint8_t* in, size_t n, uint8_t* out) {
    size_t o = 1, codeIdx = 0;
    uint8_t code = 1;
    for (size_t i = 0; i < n; i++) {
        if (in[i] == 0) {
            out[codeIdx] = code;
            codeIdx = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[codeIdx] = code;
                codeIdx = o++;
                code = 1;
            }
        }
    }
    out[codeIdx] = code;
    return o;
}

// returns decoded length or -1
int cobsDecode(const uint8_t* in, size_t n, uint8_t* out, size_t cap) {
    size_t o = 0, i = 0;
    while (i < n) {
        uint8_t code = in[i++];
        if (code == 0) return -1;
        for (uint8_t k = 1; k < code; k++) {
            if (i >= n || o >= cap) return -1;
            out[o++] = in[i++];
        }
        if (code != 0xFF && i < n) {
            if (o >= cap) return -1;
            out[o++] = 0;
        }
    }
    return (int)o;
}

// decoded = SKIND, network frame, CRC LE. returns encoded length incl. delimiter.
size_t buildStreamFrame(const uint8_t* netFrame, size_t len, uint8_t* wire) {
    uint8_t dec[260];
    dec[0] = 0x01;
    memcpy(dec + 1, netFrame, len);
    uint16_t c = crc16(dec, 1 + len);
    dec[1 + len] = c & 0xFF;
    dec[2 + len] = c >> 8;
    size_t n = cobsEncode(dec, 3 + len, wire);
    wire[n++] = 0;
    return n;
}

constexpr uint32_t kSchemaHash = 3484921581u;  // ExVectrPackets schema hash seen on the bench (any value would do)

size_t buildHeartbeat(uint8_t* wire) {
    uint8_t f[15];
    uint32_t up = millis();
    f[0] = 0x56; f[1] = 3; f[2] = 0xFF; f[3] = 1;  // CTRL, TTL, DST broadcast, SRC node 1
    f[4] = 1; f[5] = 1;                            // role vehicle, network version 1
    memcpy(f + 6, &kSchemaHash, 4);                // little-endian on Teensy
    memcpy(f + 10, &up, 4);
    f[14] = 0;
    return buildStreamFrame(f, 15, wire);
}

bool selfTest() {
    // docs/network-design.md 4.8 vector: vehicle heartbeat, hash 0x12345678, uptime 1000 ms
    static const uint8_t net[15] = {0x56, 0x03, 0xFF, 0x01, 0x01, 0x01, 0x78, 0x56, 0x34, 0x12, 0xE8, 0x03, 0, 0, 0};
    static const uint8_t expect[20] = {0x0E, 0x01, 0x56, 0x03, 0xFF, 0x01, 0x01, 0x01, 0x78, 0x56,
                                       0x34, 0x12, 0xE8, 0x03, 0x01, 0x01, 0x03, 0x1B, 0x3E, 0x00};
    uint8_t wire[64];
    size_t n = buildStreamFrame(net, 15, wire);
    return n == 20 && memcmp(wire, expect, 20) == 0;
}

// ---- per-pin receive statistics -------------------------------------------------------------------------

struct Stats {
    uint32_t bytes, zeros, frames, badCrc, badCobs, heartbeats, otherFrames;
    uint32_t srcSeen;  // bitmask of source nodes of valid frames
    uint32_t hbSrcSeen;
    uint8_t first[16];
    uint8_t nFirst;
    uint8_t buf[300];
    size_t len;
    bool overflow;
};
Stats stats[kUarts];

void feed(Stats& s, uint8_t b) {
    s.bytes++;
    if (s.nFirst < sizeof(s.first)) s.first[s.nFirst++] = b;
    if (b != 0) {
        if (s.len < sizeof(s.buf)) s.buf[s.len++] = b; else { s.overflow = true; }
        return;
    }
    s.zeros++;
    if (s.len > 0 && !s.overflow) {
        uint8_t dec[300];
        int n = cobsDecode(s.buf, s.len, dec, sizeof(dec));
        if (n < 7 || n > 243) {
            s.badCobs++;
        } else {
            uint16_t c = (uint16_t)(dec[n - 2] | (dec[n - 1] << 8));
            if (c != crc16(dec, n - 2)) s.badCrc++;
            else if (dec[0] != 0x01) s.badCobs++;
            else {
                s.frames++;
                uint8_t ctrl = dec[1], src = dec[4];
                if (src < 32) s.srcSeen |= 1u << src;
                if (((ctrl >> 4) & 3) == 1) {
                    s.heartbeats++;
                    if (src < 32) s.hbSrcSeen |= 1u << src;
                } else s.otherFrames++;
            }
        }
    } else if (s.len > 0) s.badCobs++;
    s.len = 0;
    s.overflow = false;
}

void doListen(uint32_t baud, uint32_t ms) {
    memset(stats, 0, sizeof(stats));
    for (size_t i = 0; i < kUarts; i++) {
        uarts[i].port->begin(baud, SERIAL_8N1);
        pinMode(uarts[i].txPin, INPUT);  // release the TX pad immediately (see header)
    }
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        for (size_t i = 0; i < kUarts; i++)
            while (uarts[i].port->available()) feed(stats[i], (uint8_t)uarts[i].port->read());
    }
    for (size_t i = 0; i < kUarts; i++) uarts[i].port->end();
    for (size_t i = 0; i < kUarts; i++) {
        pinMode(uarts[i].rxPin, INPUT);
        pinMode(uarts[i].txPin, INPUT);
    }
    for (size_t i = 0; i < kUarts; i++) {
        const Stats& s = stats[i];
        Serial.printf("RXPIN baud=%lu pin=%u uart=%s bytes=%lu zeros=%lu frames=%lu hb=%lu hb_src_mask=0x%lx src_mask=0x%lx "
                      "bad_crc=%lu bad_cobs=%lu first=",
                      (unsigned long)baud, uarts[i].rxPin, uarts[i].name, (unsigned long)s.bytes, (unsigned long)s.zeros,
                      (unsigned long)s.frames, (unsigned long)s.heartbeats, (unsigned long)s.hbSrcSeen,
                      (unsigned long)s.srcSeen, (unsigned long)s.badCrc, (unsigned long)s.badCobs);
        for (uint8_t k = 0; k < s.nFirst; k++) Serial.printf("%02X", s.first[k]);
        Serial.println();
    }
    Serial.println("LISTEN done");
}

void doEdges(uint32_t ms) {
    for (int pin = 0; pin <= 33; pin++) {
        pinMode(pin, INPUT);  // no pull: do not bias a node output
        int last = digitalRead(pin);
        uint32_t edges = 0, highs = 0, samples = 0;
        uint32_t t0 = millis();
        while (millis() - t0 < ms) {
            for (int k = 0; k < 2000; k++) {
                int v = digitalRead(pin);
                if (v != last) { edges++; last = v; }
                highs += v;
                samples++;
            }
        }
        Serial.printf("EDGE pin=%d edges=%lu high_pct=%lu\n", pin, (unsigned long)edges,
                      (unsigned long)(samples ? (100ull * highs / samples) : 0));
    }
    Serial.println("EDGE done");
}

void doCapture(int pin, uint32_t ms) {
    static uint32_t ts[6000];
    pinMode(pin, INPUT);
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    int last = digitalRead(pin);
    uint32_t t0ms = millis();
    size_t n = 0;
    uint32_t first = 0;
    int firstLevel = last;
    while (millis() - t0ms < ms && n < 6000) {
        int v = digitalRead(pin);
        if (v != last) {
            uint32_t now = ARM_DWT_CYCCNT;
            if (n == 0) first = now;
            ts[n++] = now - first;
            last = v;
        }
    }
    Serial.printf("CAPTURE pin=%d cpu_hz=%lu start_level=%d edges=%u\n", pin, (unsigned long)F_CPU_ACTUAL, firstLevel,
                  (unsigned)n);
    for (size_t i = 0; i < n; i += 20) {
        Serial.print("CAP ");
        for (size_t k = i; k < n && k < i + 20; k++) Serial.printf("%lu ", (unsigned long)ts[k]);
        Serial.println();
    }
    Serial.println("CAPTURE done");
}

// Pad 31 = GPIO_EMC_36 = fast GPIO8 bit 22 on the Teensy 4.0 (checked against CORE_PIN31_* at compile time).
static_assert(CORE_PIN31_BIT == 22, "pad 31 bit");

void doBitBang(int pin, uint32_t ms) {
    if (pin != 31) { Serial.printf("ERR bit-bang supports pad 31 only\n"); return; }
    if (isServoPin(pin) || isExcluded(pin)) { Serial.printf("ERR pin %d refused\n", pin); return; }
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    pinMode(31, INPUT);       // GPIO, high-Z
    GPIO8_DR_CLEAR = 1u << 22;  // output latch low; the pad is only driven when GDIR is set
    const uint32_t bitCyc = F_CPU_ACTUAL / 460800;
    Serial.printf("BB pin 31 start t=%lu bit_cycles=%lu\n", (unsigned long)millis(), (unsigned long)bitCyc);
    uint8_t wire[64];
    uint32_t t0 = millis(), next = t0;
    while (millis() - t0 < ms) {
        if ((int32_t)(millis() - next) < 0) continue;
        next += 100;
        size_t n = buildHeartbeat(wire);
        __disable_irq();
        uint32_t t = ARM_DWT_CYCCNT;
        for (size_t i = 0; i < n; i++) {
            for (int bit = -1; bit < 9; bit++) {  // start, 8 data LSB first, stop
                int lvl = (bit < 0) ? 0 : (bit == 8) ? 1 : ((wire[i] >> bit) & 1);
                if (lvl) GPIO8_GDIR &= ~(1u << 22); else GPIO8_GDIR |= (1u << 22);
                t += bitCyc;
                while ((int32_t)(ARM_DWT_CYCCNT - t) < 0) {}
            }
        }
        GPIO8_GDIR &= ~(1u << 22);
        __enable_irq();
    }
    pinMode(31, INPUT);
    Serial.printf("BB pin 31 stop t=%lu\n", (unsigned long)millis());
}

void doTransmit(int pin, uint32_t ms) {
    Uart* u = nullptr;
    for (auto& x : uarts) if (x.txPin == pin) u = &x;
    if (!u) { Serial.printf("ERR pin %d is not a UART TX pin\n", pin); return; }
    if (isServoPin(pin)) { Serial.printf("ERR pin %d is a servo/ESC pin, refused\n", pin); return; }
    if (isExcluded(pin)) { Serial.printf("ERR pin %d is excluded, refused\n", pin); return; }
    u->port->begin(460800, SERIAL_8N1);
    pinMode(u->rxPin, INPUT);  // the partner RX pad stays an input
    Serial.printf("TX pin %d start (%s) t=%lu\n", pin, u->name, (unsigned long)millis());
    uint32_t t0 = millis(), next = t0;
    uint8_t wire[64];
    while (millis() - t0 < ms) {
        if ((int32_t)(millis() - next) >= 0) {
            size_t n = buildHeartbeat(wire);
            u->port->write(wire, n);
            next += 100;
        }
    }
    u->port->flush();
    u->port->end();
    pinMode(u->txPin, INPUT);
    Serial.printf("TX pin %d stop t=%lu\n", pin, (unsigned long)millis());
}

char line[64];
size_t lineLen = 0;

void handle(char* l) {
    char c = l[0];
    long a = 0, b = 0;
    int n = sscanf(l + 1, "%ld %ld", &a, &b);
    if (c == 'V') {
        Serial.printf("PINSCAN v1 selftest=%s\n", selfTest() ? "ok" : "FAIL");
    } else if (c == 'L' && n == 2) {
        doListen((uint32_t)a, (uint32_t)b);
    } else if (c == 'E' && n == 1) {
        doEdges((uint32_t)a);
    } else if (c == 'C' && n == 2) {
        doCapture((int)a, (uint32_t)b);
    } else if (c == 'B' && n == 2) {
        doBitBang((int)a, (uint32_t)b);
    } else if (c == 'T' && n == 2) {
        doTransmit((int)a, (uint32_t)b);
    } else if (c == 'X' && n >= 1) {
        if (nExcluded < sizeof(excluded)) excluded[nExcluded++] = (uint8_t)a;
        Serial.printf("EXCLUDED %ld\n", a);
    } else {
        Serial.println("ERR unknown command");
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    // All pins are left in their power-on state (inputs with the default pull) until a command runs.
}

void loop() {
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (lineLen) { line[lineLen] = 0; handle(line); lineLen = 0; }
        } else if (lineLen < sizeof(line) - 1) line[lineLen++] = ch;
    }
}
