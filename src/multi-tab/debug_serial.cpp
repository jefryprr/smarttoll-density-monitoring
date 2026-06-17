#include "debug_serial.h"
#include "globals.h"
#include "config.h"

void serialPrintAll() {
  Serial.println(F("================================================"));
  Serial.print(F("[LOOP #")); Serial.print(loopCount); Serial.println(F("]"));

  // ── Data sensor ──
  Serial.println(F("  [Sensor]"));
  Serial.print(F("    S1 (Masuk) : "));
  Serial.print(dist1, 2); Serial.print(F(" cm | Det: ")); Serial.print(detect1);
  Serial.print(F(" | Debounced: ")); Serial.println(detect1_debounced ? F("YA") : F("tidak"));
  Serial.print(F("    S2 (Tengah): "));
  Serial.print(dist2, 2); Serial.print(F(" cm | Det: ")); Serial.println(detect2);
  Serial.print(F("    S3 (Keluar): "));
  Serial.print(dist3, 2); Serial.print(F(" cm | Det: ")); Serial.println(detect3);

  // ── Variabel Mikroskopis ──
  Serial.println(F("  [Microscopic Variables]"));

  unsigned long now_serial = millis();

  // ── TT1 (S1→S2) ──
  Serial.print(F("    TT1 [S1->S2] = ")); Serial.print(travel_time_ms, 0);
  Serial.print(F(" ms"));
  if (tt_count > 0) {
    Serial.print(F(" | LIVE - "));
    Serial.print(tt_count);
    Serial.println(F(" kendaraan di jalur S1->S2"));
  } else {
    Serial.println(F(" | (idle)"));
  }

  Serial.print(F("    TT1 Buffer  : ["));
  for (uint8_t i = 0; i < tt_count; i++) {
    uint8_t idx      = (tt_head + i) % TT_QUEUE_CAPACITY;
    unsigned long el = now_serial - tt_queue[idx];
    if (el > TT_MAX_MS) el = TT_MAX_MS;
    Serial.print(el);
    Serial.print(F("ms"));
    if (i < tt_count - 1) Serial.print(F(", "));
  }
  Serial.print(F("] ("));
  Serial.print(tt_count);
  Serial.print(F("/"));
  Serial.print(TT_QUEUE_CAPACITY);
  Serial.println(F(" slot)"));

  // ── TT2 (S2→S3) ──
  Serial.print(F("    TT2 [S2->S3] = ")); Serial.print(travel_time2_ms, 0);
  Serial.print(F(" ms"));
  if (tt2_count > 0) {
    Serial.print(F(" | LIVE - "));
    Serial.print(tt2_count);
    Serial.println(F(" kendaraan di jalur S2->S3"));
  } else {
    Serial.println(F(" | (idle)"));
  }

  Serial.print(F("    TT2 Buffer  : ["));
  for (uint8_t i = 0; i < tt2_count; i++) {
    uint8_t idx       = (tt2_head + i) % TT2_QUEUE_CAPACITY;
    unsigned long el2 = now_serial - tt2_queue[idx];
    if (el2 > TT_MAX_MS) el2 = TT_MAX_MS;
    Serial.print(el2);
    Serial.print(F("ms"));
    if (i < tt2_count - 1) Serial.print(F(", "));
  }
  Serial.print(F("] ("));
  Serial.print(tt2_count);
  Serial.print(F("/"));
  Serial.print(TT2_QUEUE_CAPACITY);
  Serial.println(F(" slot)"));

  // ── tt_fis_input (dominan) ──
  Serial.print(F("    TT_FIS (MAX) = ")); Serial.print(tt_fis_input, 0);
  Serial.print(F(" ms -> dominan: "));
  if (travel_time2_ms > travel_time_ms) {
    Serial.println(F("TT2 [S2->S3]"));
  } else if (travel_time_ms > 0.0f) {
    Serial.println(F("TT1 [S1->S2]"));
  } else {
    Serial.println(F("keduanya 0 (idle)"));
  }

  // ── Occupancy Time ──
  Serial.print(F("    OT (MAX) = ")); Serial.print(occupancy_time_ms, 0);
  Serial.print(F(" ms (= ")); Serial.print(occupancy_time_ms / 1000.0f, 2); Serial.println(F(" s)"));
  {
    unsigned long now2 = millis();
    float ot1r = occ1_active ? (float)(now2 - occ1_start_ms) : 0.0f;
    float ot2r = occ2_active ? (float)(now2 - occ2_start_ms) : 0.0f;
    float ot3r = occ3_active ? (float)(now2 - occ3_start_ms) : 0.0f;
    Serial.print(F("           OT_S1=")); Serial.print(ot1r, 0);
    Serial.print(F(" ms  OT_S2=")); Serial.print(ot2r, 0);
    Serial.print(F(" ms  OT_S3=")); Serial.print(ot3r, 0); Serial.println(F(" ms"));
  }

  // ── Output akhir ──
  Serial.println(F("  [Output Sistem]"));
  Serial.print(F("    CL (Congestion Level) = ")); Serial.println(CL, 2);
  Serial.print(F("    STATUS = "));
  switch (status) {
    case 0: Serial.println(F("LANCAR")); break;
    case 1: Serial.println(F("PADAT !")); break;
    case 2: Serial.println(F("MACET (PALANG DITUTUP)")); break;
  }

  // ── Status Servo ──
  Serial.print(F("    ServoMasuk = "));
  if (isGateOpen) {
    if (detect1_debounced && status != 2) {
      Serial.println(F("BUKA  [Kendaraan stabil di S1 & tidak macet -> izin masuk]"));
    } else {
      unsigned long elapsed_c = millis() - lastGateOpenTime;
      unsigned long sisa = (GATE_CLEARANCE_MS > elapsed_c)
                           ? (GATE_CLEARANCE_MS - elapsed_c) : 0UL;
      Serial.print(F("BUKA  [Clearance aktif, sisa ~"));
      Serial.print(sisa); Serial.println(F(" ms sebelum tutup]"));
    }
  } else {
    if (status == 2 && detect1_debounced) {
      Serial.println(F("TUTUP [MACET -> kendaraan baru DIBLOKIR]"));
    } else {
      Serial.println(F("TUTUP [Tidak ada kendaraan stabil / kondisi aman]"));
    }
  }

  Serial.println(F("================================================"));
  Serial.println();
}
