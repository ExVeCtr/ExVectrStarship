// Aero and plant timing bench firmware (NOT flight firmware, env teensy40_aerotiming; links no flight code).
//
// Measures with the DWT cycle counter, on the Teensy 4.0, the cost of
//   - starshipAero with the full (24 strip) and the reduced (8 strip) config,
//   - StarshipPlant::step (vehicle variant) with 24 strips, 8 strips, and 24 strips with gusts,
// for 16 input points that cycle through axial, broadside, tumbling and mixed-flap cases. Every call is timed on its
// own with interrupts disabled; the table gives min / mean / max cycles and microseconds over N calls, plus the first
// (cold flash cache) call of each case. The code is placed exactly as in the flight build (SIM_COLD: flash).
// Usage: flash, open the USB serial port; the table is printed every 3 s (and once after any received byte).
// Section 6.3 / 5.5 of docs/project/simulation-architecture.md hold the numbers measured with this tool.
#include <Arduino.h>
#include <math.h>

#include "sim/starship_plant.hpp"

using namespace VCTR;
using ShipSim::AeroConfig;
using ShipSim::AeroInput;
using ShipSim::AeroOutput;
using ShipSim::StarshipPlant;
using ShipSim::StarshipPlantParams;

namespace {

constexpr int kPoints = 16;
constexpr int kCalls = 800; // per case (multiple of kPoints)

struct Pt {
  float vel[3], omega[3], flap[4];
};
Pt pts[kPoints];

struct Result {
  uint32_t first, minC, maxC;
  uint64_t sum;
};

volatile float sink;

void buildPoints() {
  for (int i = 0; i < kPoints; i++) {
    const float al = (i * 180.0f / (kPoints - 1)) * 0.0174533f; // alpha 0..180 deg
    const float sp = 1.0f + 14.0f * float((i * 7) % kPoints) / float(kPoints - 1);
    const float ph = i * 0.9f;
    pts[i].vel[0] = sp * sinf(al) * cosf(ph);
    pts[i].vel[1] = sp * sinf(al) * sinf(ph);
    pts[i].vel[2] = sp * cosf(al);
    pts[i].omega[0] = (i % 3 == 0) ? 0.0f : 1.5f * sinf(ph);
    pts[i].omega[1] = (i % 3 == 0) ? 0.0f : -1.0f * cosf(ph);
    pts[i].omega[2] = (i % 5 == 0) ? 0.5f : 0.0f;
    for (int k = 0; k < 4; k++) pts[i].flap[k] = ((i + 3 * k) % 6) * 0.3141f; // 0..90 deg, mixed
  }
}

inline uint32_t cycles() { return ARM_DWT_CYCCNT; }

void aeroCase(const AeroConfig &cfg, Result &r) {
  r.minC = 0xffffffffu;
  r.maxC = 0;
  r.sum = 0;
  for (int n = 0; n < kCalls; n++) {
    const Pt &p = pts[n % kPoints];
    AeroInput in;
    in.airVelBody = {p.vel[0], p.vel[1], p.vel[2]};
    in.omegaBody = {p.omega[0], p.omega[1], p.omega[2]};
    for (int k = 0; k < 4; k++) in.flapAngle[k] = p.flap[k];
    in.rho = 1.2f;
    AeroOutput out;
    __disable_irq();
    const uint32_t t0 = cycles();
    ShipSim::starshipAero(cfg, in, out);
    const uint32_t dt = cycles() - t0;
    __enable_irq();
    sink = out.total.force(0) + out.total.torque(1);
    if (n == 0) r.first = dt;
    if (dt < r.minC) r.minC = dt;
    if (dt > r.maxC) r.maxC = dt;
    r.sum += dt;
  }
}

void plantCase(const StarshipPlantParams &pp, Result &r) {
  static StarshipPlant::Config cfg;
  static StarshipPlant::State x;
  cfg = StarshipPlant::configure(pp);
  r.minC = 0xffffffffu;
  r.maxC = 0;
  r.sum = 0;
  StarshipPlant::init(cfg, x);
  for (int n = 0; n < kCalls; n++) {
    const Pt &p = pts[n % kPoints];
    if (n % kPoints == 0) StarshipPlant::init(cfg, x); // keep the state bounded; not timed
    x.body.att(0) = p.omega[0];
    x.body.att(1) = p.omega[1];
    x.body.att(2) = p.omega[2];
    x.body.pos(0) = -p.vel[0];
    x.body.pos(1) = -p.vel[1];
    x.body.pos(2) = -p.vel[2];
    x.body.pos(5) = 20.0f;
    StarshipPlant::Input u;
    u.thrustBody = {0, 0, 0};
    u.rollTorque = 0.0f;
    u.thrustEnabled = false;
    for (int k = 0; k < 4; k++) u.flapAngle[k] = p.flap[k];
    StarshipPlant::Output y;
    Dyn::NoProbe pr;
    __disable_irq();
    const uint32_t t0 = cycles();
    StarshipPlant::step(cfg, x, u, 0.002f, y, pr);
    const uint32_t dt = cycles() - t0;
    __enable_irq();
    sink = y.aero.force(0) + x.body.pos(5);
    if (n == 0) r.first = dt;
    if (dt < r.minC) r.minC = dt;
    if (dt > r.maxC) r.maxC = dt;
    r.sum += dt;
  }
}

void printRow(const char *name, const Result &r) {
  const float mean = float(r.sum) / float(kCalls);
  const float us = 1e6f / float(F_CPU_ACTUAL);
  Serial.printf("%-34s %7lu %9.0f %7lu %6lu | %7.2f %7.2f %7.2f %7.2f\n", name, (unsigned long)r.minC, mean,
                (unsigned long)r.maxC, (unsigned long)r.first, r.minC * us, mean * us, r.maxC * us, r.first * us);
}

void runAll() {
  StarshipPlantParams full;                 // nominal, 24 strips
  StarshipPlantParams red = full;
  red.hull.strips = 8;
  StarshipPlantParams gust = full;
  gust.wind.speed_mps = 3.0f;
  gust.wind.gustStrength_mps = 2.0f;

  static AeroConfig cFull, cRed;
  cFull = ShipSim::makeAeroConfig(full);
  cRed = ShipSim::makeReducedAeroConfig(full, 8);

  Result r[5];
  aeroCase(cFull, r[0]);
  aeroCase(cRed, r[1]);
  plantCase(full, r[2]);
  plantCase(red, r[3]);
  plantCase(gust, r[4]);

  Serial.printf("\nAEROTIMING F_CPU=%lu Hz, %d calls per case, %d input points, interrupts off per call\n",
                (unsigned long)F_CPU_ACTUAL, kCalls, kPoints);
  Serial.println("case                                  min    mean     max  first | min_us mean_us  max_us first_us");
  printRow("starshipAero, 24 strips", r[0]);
  printRow("starshipAero, 8 strips", r[1]);
  printRow("StarshipPlant::step, 24 strips", r[2]);
  printRow("StarshipPlant::step, 8 strips", r[3]);
  printRow("StarshipPlant::step, 24 + gusts", r[4]);
  Serial.println("AEROTIMING done");
}

} // namespace

void setup() {
  Serial.begin(115200);
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  buildPoints();
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}
}

void loop() {
  static uint32_t last = 0;
  const bool rx = Serial.available() > 0;
  while (Serial.available() > 0) Serial.read();
  if (rx || millis() - last > 3000) {
    last = millis();
    runAll();
  }
}
