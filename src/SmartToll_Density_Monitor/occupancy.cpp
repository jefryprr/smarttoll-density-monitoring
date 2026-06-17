#include "occupancy.h"
#include "globals.h"
#include "config.h"

void updateOccupancyTime() {
  unsigned long now = millis();

  // ── Sensor 1 — Zona Masuk ──
  if (detect1 == 1) {
    if (!occ1_active) { occ1_start_ms = now; occ1_active = true; }
  } else {
    occ1_active = false;
  }

  // ── Sensor 2 — Zona Tengah ──
  if (detect2 == 1) {
    if (!occ2_active) { occ2_start_ms = now; occ2_active = true; }
  } else {
    occ2_active = false;
  }

  // ── Sensor 3 — Zona Keluar ──
  if (detect3 == 1) {
    if (!occ3_active) { occ3_start_ms = now; occ3_active = true; }
  } else {
    occ3_active = false;
  }

  // ── Hitung durasi blokir saat ini per sensor (dengan clamping) ──
  unsigned long raw1 = occ1_active ? (now - occ1_start_ms) : 0UL;
  unsigned long raw2 = occ2_active ? (now - occ2_start_ms) : 0UL;
  unsigned long raw3 = occ3_active ? (now - occ3_start_ms) : 0UL;

  float ot1 = (raw1 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw1;
  float ot2 = (raw2 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw2;
  float ot3 = (raw3 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw3;

  // ── OT untuk FIS = MAX dari ketiga sensor ──
  float max12 = (ot1 > ot2) ? ot1 : ot2;
  occupancy_time_ms = (max12 > ot3) ? max12 : ot3;
}
