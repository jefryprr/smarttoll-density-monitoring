/**
 * debug_serial.cpp — Implementasi Fungsi Debug Serial
 * SmartToll Density Monitoring System v3.2.1
 */

#include "debug_serial.h"
#include "fuzzy_logic.h"
#include "sensors.h"

// Helper untuk status string
static const char* statusStr(StatusKemacetan st) {
    switch (st) {
        case LANCAR: return "LANCAR";
        case PADAT:  return "PADAT";
        case MACET:  return "MACET";
        default:     return "UNKNOWN";
    }
}

// ============================================================
// Cetak Semua Data Debug
// ============================================================
void serialPrintAll(unsigned long loop_num) {
    Serial.println(F(""));
    Serial.println(F("============================================================"));
    Serial.print(F("LOOP #"));
    Serial.println(loop_num);
    Serial.println(F("============================================================"));

    // ----------------------------------------------------------
    // 1. Pembacaan Sensor
    // ----------------------------------------------------------
    Serial.println(F("[SENSOR READINGS]"));
    Serial.print(F("  S1: dist="));
    Serial.print(dist1, 1);
    Serial.print(F(" cm | detect="));
    Serial.print(detect1);
    Serial.print(F(" | debounced="));
    Serial.println(debounced1);

    Serial.print(F("  S2: dist="));
    Serial.print(dist2, 1);
    Serial.print(F(" cm | detect="));
    Serial.println(detect2);

    Serial.print(F("  S3: dist="));
    Serial.print(dist3, 1);
    Serial.print(F(" cm | detect="));
    Serial.println(detect3);

    // ----------------------------------------------------------
    // 2. Variabel Mikroskopik
    // ----------------------------------------------------------
    Serial.println(F(""));
    Serial.println(F("[VARIABEL MIKROSKOPIK]"));

    // Travel Time
    Serial.print(F("  TT1 (S1->S2): "));
    Serial.print(travel_time_ms);
    Serial.print(F(" ms | Buffer count="));
    Serial.println(tt_count);

    Serial.print(F("  TT2 (S2->S3): "));
    Serial.print(travel_time2_ms);
    Serial.print(F(" ms | Buffer count="));
    Serial.println(tt2_count);

    Serial.print(F("  TT FIS Input (MAX): "));
    Serial.print(tt_fis_input, 1);
    Serial.println(F(" ms"));

    // Occupancy Time
    Serial.print(F("  OT S1="));
    Serial.print(ot_s1);
    Serial.print(F(" ms | S2="));
    Serial.print(ot_s2);
    Serial.print(F(" ms | S3="));
    Serial.print(ot_s3);
    Serial.print(F(" ms | MAX="));
    Serial.print(occupancy_time_ms);
    Serial.println(F(" ms"));

    // Rising edge
    Serial.print(F("  Rising Edge: S1="));
    Serial.print(isRisingEdge(detect1, prev_detect1));
    Serial.print(F(" S2="));
    Serial.print(isRisingEdge(detect2, prev_detect2));
    Serial.print(F(" S3="));
    Serial.println(isRisingEdge(detect3, prev_detect3));

    // ----------------------------------------------------------
    // 3. Fuzzifikasi FIS
    // ----------------------------------------------------------
    Serial.println(F(""));
    Serial.println(F("[FUZZIFIKASI FIS]"));

    float tt = tt_fis_input;
    float ot = (float)occupancy_time_ms;

    Serial.print(F("  TT="));
    Serial.print(tt, 1);
    Serial.print(F(" ms => Cepat="));
    Serial.print(tt_cepat(tt), 3);
    Serial.print(F(" Sedang="));
    Serial.print(tt_sedang(tt), 3);
    Serial.print(F(" Lama="));
    Serial.println(tt_lama(tt), 3);

    Serial.print(F("  OT="));
    Serial.print(ot, 1);
    Serial.print(F(" ms => Singkat="));
    Serial.print(ot_singkat(ot), 3);
    Serial.print(F(" Sedang="));
    Serial.print(ot_sedang(ot), 3);
    Serial.print(F(" Lama="));
    Serial.println(ot_lama(ot), 3);

    // ----------------------------------------------------------
    // 4. Firing Strength Aturan
    // ----------------------------------------------------------
    Serial.println(F(""));
    Serial.println(F("[FIRING STRENGTH ATURAN]"));

    float tt_c = tt_cepat(tt);
    float tt_s = tt_sedang(tt);
    float tt_l = tt_lama(tt);
    float ot_sg = ot_singkat(ot);
    float ot_sd = ot_sedang(ot);
    float ot_lm = ot_lama(ot);

    Serial.print(F("  R1 (Cepat*Singkat->Lancar)  = min("));
    Serial.print(tt_c, 3); Serial.print(F(",")); Serial.print(ot_sg, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_c, ot_sg), 3);

    Serial.print(F("  R2 (Cepat*Sedang->Lancar)   = min("));
    Serial.print(tt_c, 3); Serial.print(F(",")); Serial.print(ot_sd, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_c, ot_sd), 3);

    Serial.print(F("  R3 (Cepat*Lama->Padat)      = min("));
    Serial.print(tt_c, 3); Serial.print(F(",")); Serial.print(ot_lm, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_c, ot_lm), 3);

    Serial.print(F("  R4 (Sedang*Singkat->Lancar) = min("));
    Serial.print(tt_s, 3); Serial.print(F(",")); Serial.print(ot_sg, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_s, ot_sg), 3);

    Serial.print(F("  R5 (Sedang*Sedang->Padat)   = min("));
    Serial.print(tt_s, 3); Serial.print(F(",")); Serial.print(ot_sd, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_s, ot_sd), 3);

    Serial.print(F("  R6 (Sedang*Lama->Macet)     = min("));
    Serial.print(tt_s, 3); Serial.print(F(",")); Serial.print(ot_lm, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_s, ot_lm), 3);

    Serial.print(F("  R7 (Lama*Singkat->Padat)    = min("));
    Serial.print(tt_l, 3); Serial.print(F(",")); Serial.print(ot_sg, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_l, ot_sg), 3);

    Serial.print(F("  R8 (Lama*Sedang->Macet)     = min("));
    Serial.print(tt_l, 3); Serial.print(F(",")); Serial.print(ot_sd, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_l, ot_sd), 3);

    Serial.print(F("  R9 (Lama*Lama->Macet)       = min("));
    Serial.print(tt_l, 3); Serial.print(F(",")); Serial.print(ot_lm, 3);
    Serial.print(F(") = ")); Serial.println(min(tt_l, ot_lm), 3);

    // Agregasi
    float r1=min(tt_c,ot_sg), r2=min(tt_c,ot_sd), r3=min(tt_c,ot_lm);
    float r4=min(tt_s,ot_sg), r5=min(tt_s,ot_sd), r6=min(tt_s,ot_lm);
    float r7=min(tt_l,ot_sg), r8=min(tt_l,ot_sd), r9=min(tt_l,ot_lm);
    float str_lancar = max(r1, max(r2, r4));
    float str_padat  = max(r3, max(r5, r7));
    float str_macet  = max(r6, max(r8, r9));

    Serial.print(F("  Agregasi Lancar=max(R1,R2,R4)="));
    Serial.println(str_lancar, 3);
    Serial.print(F("  Agregasi Padat =max(R3,R5,R7)="));
    Serial.println(str_padat, 3);
    Serial.print(F("  Agregasi Macet =max(R6,R8,R9)="));
    Serial.println(str_macet, 3);

    // ----------------------------------------------------------
    // 5. Hasil COG Defuzzifikasi
    // ----------------------------------------------------------
    Serial.println(F(""));
    Serial.println(F("[COG DEFFUZIFIKASI]"));
    Serial.print(F("  CL = "));
    Serial.println(cl, 2);

    // ----------------------------------------------------------
    // 6. Output Sistem
    // ----------------------------------------------------------
    Serial.println(F(""));
    Serial.println(F("[OUTPUT SISTEM]"));
    Serial.print(F("  Status     : "));
    Serial.println(statusStr(status));
    Serial.print(F("  Gate       : "));
    Serial.println(gateOpen ? "TERBUKA" : "TERTUTUP");
    Serial.print(F("  Vehicle N  : "));
    Serial.println(vehicle_count);
    Serial.print(F("  LED Hijau  : "));
    Serial.println(digitalRead(LED_HIJAU));
    Serial.print(F("  LED Kuning : "));
    Serial.println(digitalRead(LED_KUNING));
    Serial.print(F("  LED Merah  : "));
    Serial.println(digitalRead(LED_MERAH));
    Serial.print(F("  Buzzer     : "));
    Serial.println(digitalRead(BUZZER_PIN));

    Serial.println(F("============================================================"));
}
