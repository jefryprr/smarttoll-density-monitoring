/**
 * ================================================================
 *  SmartToll Density Monitoring System — v3.2.1 (ESP32 Port)
 *  Tugas Besar Sistem Kendali dan Sensor Aktuator
 * ================================================================
 *  Dokumentasi lengkap (wiring, rule base, changelog) ada di
 *  README.md pada root repo ini.
 *
 *  Paradigma: Discrete Event System (DES) mikroskopis.
 *    INPUT 1 (TT) : tt_fis_input = MAX(TT1[S1->S2], TT2[S2->S3])
 *    INPUT 2 (OT) : occupancy_time_ms = MAX(OT_S1, OT_S2, OT_S3)
 *    OUTPUT       : CL (Congestion Level) via FIS Mamdani 9-rule
 *
 *  TT panjang/timeout -> kendaraan lambat/mogok di titik buta.
 *  OT panjang          -> kendaraan berhenti tepat di depan sensor.
 *
 *  Deteksi blind spot (Dual FIFO Travel Time):
 *    TT1 melacak segmen S1->S2, TT2 melacak segmen S2->S3.
 *    tt_fis_input = MAX(TT1, TT2) -> segmen paling buruk yang
 *    menentukan output FIS, sehingga Rule R7 (TT=Lama & OT=Singkat
 *    -> MACET) otomatis mencakup kedua blind spot tanpa rule baru.
 *
 *  Komponen aktif:
 *    - 3x HC-SR04  -> S1 (masuk), S2 (tengah), S3 (keluar)
 *    - 1x LCD 16x2 I2C (0x27) -- SDA: GPIO21, SCL: GPIO22
 *    - 3x LED (Hijau 18 / Kuning 19 / Merah 23)
 *    - 1x Buzzer aktif (GPIO 4)
 *    - 1x Servo SG90 -- palang masuk (GPIO 5)
 *
 *  Daftar isi file ini (cari dengan Ctrl+F):
 *    [1] LIBRARY & KONFIGURASI PIN/KONSTANTA
 *    [2] OBJEK & VARIABEL GLOBAL
 *    [3] FUNGSI SENSOR
 *    [4] FUNGSI TRAVEL TIME (DUAL FIFO)
 *    [5] FUNGSI OCCUPANCY TIME
 *    [6] FUNGSI FUZZY LOGIC (MF + COG + MAMDANI)
 *    [7] FUNGSI AKTUATOR (LED, BUZZER, SERVO)
 *    [8] FUNGSI DISPLAY (LCD)
 *    [9] FUNGSI DEBUG SERIAL
 *    [10] SETUP
 *    [11] LOOP UTAMA
 * ================================================================
 */

// ╔══════════════════════════════════════════════════════════════╗
// ║  [1] LIBRARY & KONFIGURASI PIN / KONSTANTA                  ║
// ╚══════════════════════════════════════════════════════════════╝

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>   // Ganti dari <Servo.h> untuk ESP32

// ---------------- Pin Sensor Ultrasonik HC-SR04 ----------------
#define TRIG1  13   // Sensor 1 — Zona Masuk (sebelum palang)
#define ECHO1  12
#define TRIG2  14   // Sensor 2 — Zona Tengah (titik ukur TT1 & TT2)
#define ECHO2  27
#define TRIG3  26   // Sensor 3 — Zona Keluar (titik akhir TT2)
#define ECHO3  25

// ---------------- Pin Aktuator ----------------
#define SERVO_MASUK_PIN  5    // Palang masuk

#define LED_HIJAU   18
#define LED_KUNING  19
#define LED_MERAH   23

#define BUZZER_PIN  4

// ---------------- Parameter Sensor & Timing Loop ----------------
#define BATAS_LEBAR_JALAN   12.0f   // [cm] ambang deteksi kendaraan
#define LOOP_DELAY          50      // [ms] target durasi satu siklus loop
#define SENSOR_DELAY        15      // [ms] jeda antar pembacaan sensor (anti cross-talk)

// ---------------- Parameter Debounce & Gate Clearance ----------------
#define DEBOUNCE_MS         150UL   // [ms] debounce Sensor 1
#define GATE_CLEARANCE_MS   200UL   // [ms] palang tetap terbuka sebelum menutup

// ---------------- Parameter Servo ----------------
#define SERVO_BUKA   90    // [°] posisi terbuka
#define SERVO_TUTUP   0    // [°] posisi tertutup (default)

// ---------------- Threshold Klasifikasi Status (dari CL) ----------------
#define CL_BATAS_LANCAR  34.0f
#define CL_BATAS_PADAT   67.0f

// ---------------- Domain Variabel Fuzzy ----------------
// TT (Travel Time) berlaku untuk TT1 [S1→S2] maupun TT2 [S2→S3].
#define TT_MAX_MS  10000UL
// OT (Occupancy Time): durasi sensor terbaca HIGH secara kontinu.
#define OT_MAX_MS  5000UL

// ---------------- Kapasitas Buffer FIFO ----------------
#define TT_QUEUE_CAPACITY   5   // FIFO TT1 (S1→S2)
#define TT2_QUEUE_CAPACITY  5   // FIFO TT2 (S2→S3)


// ╔══════════════════════════════════════════════════════════════╗
// ║  [2] OBJEK & VARIABEL GLOBAL                                 ║
// ╚══════════════════════════════════════════════════════════════╝

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMasuk;

// --- Data Sensor & Deteksi ---
float dist1 = 0.0f, dist2 = 0.0f, dist3 = 0.0f;
int   detect1 = 0, detect2 = 0, detect3 = 0;
int   prev_detect1 = 0, prev_detect2 = 0, prev_detect3 = 0;

// --- Debounce Sensor 1 ---
unsigned long detect1_stable_since = 0;
bool          detect1_debounced    = false;

// --- FIFO Travel Time TT1 (S1→S2) ---
unsigned long tt_queue[TT_QUEUE_CAPACITY];
uint8_t       tt_head  = 0;
uint8_t       tt_tail  = 0;
uint8_t       tt_count = 0;
float         travel_time_ms = 0.0f;

// --- FIFO Travel Time TT2 (S2→S3) ---
unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
uint8_t       tt2_head  = 0;
uint8_t       tt2_tail  = 0;
uint8_t       tt2_count = 0;
float         travel_time2_ms = 0.0f;

// --- Input FIS Gabungan: tt_fis_input = MAX(TT1, TT2) ---
float tt_fis_input = 0.0f;

// --- Occupancy Time ---
unsigned long occ1_start_ms = 0, occ2_start_ms = 0, occ3_start_ms = 0;
bool          occ1_active   = false, occ2_active = false, occ3_active = false;
float         occupancy_time_ms = 0.0f;

// --- Output Fuzzy & Status Akhir ---
float CL     = 0.0f;
int   status = 0;

// --- State Machine Servo Masuk ---
unsigned long lastGateOpenTime = 0;
bool          isGateOpen       = false;

// --- Timing Non-blocking LED/Buzzer ---
unsigned long lastBlink     = 0;
bool          ledMerahState = false;
unsigned long lastBuzzer    = 0;

// --- Monitoring ---
unsigned long loopCount = 0;


// ╔══════════════════════════════════════════════════════════════╗
// ║  [3] FUNGSI SENSOR                                           ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * Mengukur jarak satu sensor HC-SR04 (Time-of-Flight).
 * Rumus: d [cm] = t_echo [µs] × 0.01715
 * Return 30.0 cm jika timeout (tidak ada pantulan terdeteksi).
 */
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 30.0f;
  return (float)duration * 0.01715f;
}

/** Membaca ketiga sensor berurutan (time-multiplexed) untuk hindari cross-talk. */
void readAllSensors() {
  dist1 = readUltrasonic(TRIG1, ECHO1);
  delay(SENSOR_DELAY);
  dist2 = readUltrasonic(TRIG2, ECHO2);
  delay(SENSOR_DELAY);
  dist3 = readUltrasonic(TRIG3, ECHO3);
  delay(SENSOR_DELAY);
}

/** Return 1 jika kendaraan terdeteksi di depan sensor, 0 jika tidak. */
int detectVehicle(float distance_cm) {
  return (distance_cm < BATAS_LEBAR_JALAN) ? 1 : 0;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [4] FUNGSI TRAVEL TIME (DUAL FIFO TT1 & TT2)                ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * Memperbarui Travel Time TT1 (S1→S2) dan TT2 (S2→S3).
 * Satu Rising Edge di S2 memicu dua aksi: POP TT1 (selesai S1→S2)
 * sekaligus PUSH TT2 (mulai S2→S3) — sehingga tidak ada gap antara
 * kedua segmen, seluruh jalur S1→S3 terpantau tanpa blind spot.
 */
void updateTravelTime() {
  unsigned long now = millis();

  // ── TT1 FASE 1: Rising Edge S1 → PUSH ke tt_queue ──
  if (detect1 == 1 && prev_detect1 == 0) {
    if (tt_count < TT_QUEUE_CAPACITY) {
      tt_queue[tt_tail] = now;
      tt_tail           = (tt_tail + 1) % TT_QUEUE_CAPACITY;
      tt_count++;
      Serial.print(F("  [TT1] Rising Edge S1 @ "));
      Serial.print(now);
      Serial.print(F(" ms -> PUSH. TT1 Antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT1] Antrean PENUH ("));
      Serial.print(TT_QUEUE_CAPACITY);
      Serial.println(F(" slot) - S1 Rising Edge diabaikan."));
    }
  }

  // ── TT1 FASE 2 + TT2 FASE 1: Rising Edge S2 ──
  if (detect2 == 1 && prev_detect2 == 0) {

    // AKSI 1 — Selesaikan TT1
    if (tt_count > 0) {
      unsigned long s1_ts = tt_queue[tt_head];
      tt_head             = (tt_head + 1) % TT_QUEUE_CAPACITY;
      tt_count--;
      float elapsed  = (float)(now - s1_ts);
      travel_time_ms = (elapsed > (float)TT_MAX_MS)
                       ? (float)TT_MAX_MS : elapsed;
      Serial.print(F("  [TT1] Rising Edge S2 -> POP. TT1 final = "));
      Serial.print(travel_time_ms, 0);
      Serial.print(F(" ms. Sisa TT1 antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT1] Rising Edge S2 tapi tt1 antrean kosong -> diabaikan."));
    }

    // AKSI 2 — Mulai TT2
    if (tt2_count < TT2_QUEUE_CAPACITY) {
      tt2_queue[tt2_tail] = now;
      tt2_tail            = (tt2_tail + 1) % TT2_QUEUE_CAPACITY;
      tt2_count++;
      Serial.print(F("  [TT2] Rising Edge S2 -> PUSH. TT2 Antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT2] Antrean PENUH ("));
      Serial.print(TT2_QUEUE_CAPACITY);
      Serial.println(F(" slot) - S2 PUSH diabaikan."));
    }
  }

  // ── TT2 FASE 2: Rising Edge S3 → POP, hitung TT2 final ──
  if (detect3 == 1 && prev_detect3 == 0) {
    if (tt2_count > 0) {
      unsigned long s2_ts = tt2_queue[tt2_head];
      tt2_head            = (tt2_head + 1) % TT2_QUEUE_CAPACITY;
      tt2_count--;
      float elapsed2  = (float)(now - s2_ts);
      travel_time2_ms = (elapsed2 > (float)TT_MAX_MS)
                        ? (float)TT_MAX_MS : elapsed2;
      Serial.print(F("  [TT2] Rising Edge S3 -> POP. TT2 final = "));
      Serial.print(travel_time2_ms, 0);
      Serial.print(F(" ms. Sisa TT2 antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT2] Rising Edge S3 tapi tt2 antrean kosong -> diabaikan."));
    }
  }

  // ── TT1 FASE 3: Live Tracking (PEEK kendaraan terdepan S1→S2) ──
  if (tt_count > 0) {
    float elapsed  = (float)(now - tt_queue[tt_head]);
    travel_time_ms = (elapsed > (float)TT_MAX_MS)
                     ? (float)TT_MAX_MS : elapsed;
  }

  // ── TT2 FASE 3: Live Tracking (PEEK kendaraan terdepan S2→S3) ──
  if (tt2_count > 0) {
    float elapsed2  = (float)(now - tt2_queue[tt2_head]);
    travel_time2_ms = (elapsed2 > (float)TT_MAX_MS)
                      ? (float)TT_MAX_MS : elapsed2;
  }

  // ── TT1 FASE 4: Idle Reset ──
  if (tt_count == 0 && detect1 == 0 && detect2 == 0 && detect3 == 0) {
    travel_time_ms = 0.0f;
  }

  // ── TT2 FASE 4: Idle Reset ──
  if (tt2_count == 0 && detect2 == 0 && detect3 == 0) {
    travel_time2_ms = 0.0f;
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [5] FUNGSI OCCUPANCY TIME                                   ║
// ╚══════════════════════════════════════════════════════════════╝

/** OT untuk FIS = MAX(OT_S1, OT_S2, OT_S3) -> kondisi terparah. */
void updateOccupancyTime() {
  unsigned long now = millis();

  if (detect1 == 1) {
    if (!occ1_active) { occ1_start_ms = now; occ1_active = true; }
  } else {
    occ1_active = false;
  }

  if (detect2 == 1) {
    if (!occ2_active) { occ2_start_ms = now; occ2_active = true; }
  } else {
    occ2_active = false;
  }

  if (detect3 == 1) {
    if (!occ3_active) { occ3_start_ms = now; occ3_active = true; }
  } else {
    occ3_active = false;
  }

  unsigned long raw1 = occ1_active ? (now - occ1_start_ms) : 0UL;
  unsigned long raw2 = occ2_active ? (now - occ2_start_ms) : 0UL;
  unsigned long raw3 = occ3_active ? (now - occ3_start_ms) : 0UL;

  float ot1 = (raw1 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw1;
  float ot2 = (raw2 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw2;
  float ot3 = (raw3 > OT_MAX_MS) ? (float)OT_MAX_MS : (float)raw3;

  float max12 = (ot1 > ot2) ? ot1 : ot2;
  occupancy_time_ms = (max12 > ot3) ? max12 : ot3;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [6] FUNGSI FUZZY LOGIC (MF + COG + MAMDANI)                 ║
// ╚══════════════════════════════════════════════════════════════╝

float triangularMF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x <= b)           return (x - a) / (b - a);
  return                       (c - x) / (c - b);
}

float trapezoidalMF(float x, float a, float b, float c, float d) {
  if (x < a || x > d)    return 0.0f;
  if (x >= b && x <= c)  return 1.0f;
  if (x < b) {
    if (fabsf(b - a) < 0.001f) return 1.0f;
    return (x - a) / (b - a);
  }
  if (fabsf(d - c) < 0.001f) return 0.0f;
  return (d - x) / (d - c);
}

// ─── MF Input TT — domain [0, 10000] ms (berlaku untuk MAX(TT1, TT2)) ───
float mf_TT_Cepat(float x)  { return trapezoidalMF(x,    0.0f,    0.0f,  1500.0f,  3000.0f); }
float mf_TT_Sedang(float x) { return triangularMF (x, 2000.0f, 4000.0f,  6000.0f); }
float mf_TT_Lama(float x)   { return trapezoidalMF(x, 5000.0f, 8000.0f, 10000.0f, 10000.0f); }

// ─── MF Input OT — domain [0, 5000] ms ───
float mf_OT_Singkat(float x) { return trapezoidalMF(x,    0.0f,    0.0f,  500.0f, 1500.0f); }
float mf_OT_Sedang(float x)  { return triangularMF (x, 1000.0f, 2000.0f, 3000.0f); }
float mf_OT_Lama(float x)    { return trapezoidalMF(x, 2500.0f, 4000.0f, 5000.0f, 5000.0f); }

// ─── MF Output CL — domain [0, 100] ───
float mf_CL_Lancar(float z) { return trapezoidalMF(z,  0.0f,  0.0f, 10.0f, 40.0f); }
float mf_CL_Padat(float z)  { return triangularMF (z, 20.0f, 50.0f, 80.0f); }
float mf_CL_Macet(float z)  { return trapezoidalMF(z, 60.0f, 80.0f, 100.0f, 100.0f); }

/**
 * Defuzzifikasi Centre of Gravity diskrit, step=5 (21 iterasi).
 * CL* ≈ Σ[z_i × μ_agg(z_i)] / Σ[μ_agg(z_i)],  z_i = 0,5,10,...,100
 */
float defuzzifyCOG(float a_lancar, float a_padat, float a_macet) {
  float numerator   = 0.0f;
  float denominator = 0.0f;

  for (int z = 0; z <= 100; z += 5) {
    float fz = (float)z;

    float mu_L = mf_CL_Lancar(fz);
    float mu_P = mf_CL_Padat(fz);
    float mu_M = mf_CL_Macet(fz);

    float clipped_L = (a_lancar < mu_L) ? a_lancar : mu_L;
    float clipped_P = (a_padat  < mu_P) ? a_padat  : mu_P;
    float clipped_M = (a_macet  < mu_M) ? a_macet  : mu_M;

    float agg = clipped_L;
    if (clipped_P > agg) agg = clipped_P;
    if (clipped_M > agg) agg = clipped_M;

    numerator   += fz * agg;
    denominator += agg;
  }

  if (denominator < 0.001f) return 0.0f;
  return numerator / denominator;
}

/**
 * Pipeline lengkap FIS Mamdani: [tt_fis_input, OT] -> CL*.
 *
 * Rule Base (9 rule, AND=MIN, agregasi output=MAX):
 *   R1 Cepat ∧Singkat->Lancar   R6 Sedang∧Lama   ->Macet
 *   R2 Cepat ∧Sedang ->Lancar   R7 Lama  ∧Singkat->Macet (blind spot)
 *   R3 Cepat ∧Lama   ->Padat    R8 Lama  ∧Sedang ->Macet
 *   R4 Sedang∧Singkat->Lancar   R9 Lama  ∧Lama   ->Macet
 *   R5 Sedang∧Sedang ->Padat
 *
 *   R7 mencakup blind spot S1-S2 (R7a, TT1 dominan) maupun S2-S3
 *   (R7b, TT2 dominan) karena tt_val = MAX(TT1, TT2).
 */
float runFuzzyMamdani(float tt_val, float ot_val) {

  // ── Langkah 1: Fuzzifikasi ──
  float mu_TT_C  = mf_TT_Cepat(tt_val);
  float mu_TT_S  = mf_TT_Sedang(tt_val);
  float mu_TT_L  = mf_TT_Lama(tt_val);

  float mu_OT_Si = mf_OT_Singkat(ot_val);
  float mu_OT_Se = mf_OT_Sedang(ot_val);
  float mu_OT_L  = mf_OT_Lama(ot_val);

  // ── Langkah 2: Evaluasi rule — firing strength (alpha = MIN) ──
  float alpha_R1 = (mu_TT_C < mu_OT_Si) ? mu_TT_C : mu_OT_Si;
  float alpha_R2 = (mu_TT_C < mu_OT_Se) ? mu_TT_C : mu_OT_Se;
  float alpha_R3 = (mu_TT_C < mu_OT_L)  ? mu_TT_C : mu_OT_L;
  float alpha_R4 = (mu_TT_S < mu_OT_Si) ? mu_TT_S : mu_OT_Si;
  float alpha_R5 = (mu_TT_S < mu_OT_Se) ? mu_TT_S : mu_OT_Se;
  float alpha_R6 = (mu_TT_S < mu_OT_L)  ? mu_TT_S : mu_OT_L;
  float alpha_R7 = (mu_TT_L < mu_OT_Si) ? mu_TT_L : mu_OT_Si;
  float alpha_R8 = (mu_TT_L < mu_OT_Se) ? mu_TT_L : mu_OT_Se;
  float alpha_R9 = (mu_TT_L < mu_OT_L)  ? mu_TT_L : mu_OT_L;

  // ── Langkah 3: Agregasi per kelas output (MAX) ──
  float alpha_Lancar = alpha_R1;
  if (alpha_R2 > alpha_Lancar) alpha_Lancar = alpha_R2;
  if (alpha_R4 > alpha_Lancar) alpha_Lancar = alpha_R4;

  float alpha_Padat = (alpha_R3 > alpha_R5) ? alpha_R3 : alpha_R5;

  float alpha_Macet = alpha_R6;
  if (alpha_R7 > alpha_Macet) alpha_Macet = alpha_R7;
  if (alpha_R8 > alpha_Macet) alpha_Macet = alpha_R8;
  if (alpha_R9 > alpha_Macet) alpha_Macet = alpha_R9;

  // ── Log detail FIS ke Serial Monitor ──
  Serial.println(F("  [FIS] --- Fuzzifikasi ---"));
  Serial.print(F("    TT_FIS(")); Serial.print(tt_val, 0); Serial.print(F(" ms): "));
  Serial.print(F("Cepat="));  Serial.print(mu_TT_C, 3);
  Serial.print(F("  Sedang=")); Serial.print(mu_TT_S, 3);
  Serial.print(F("  Lama="));  Serial.println(mu_TT_L, 3);
  Serial.print(F("    OT(")); Serial.print(ot_val, 0); Serial.print(F(" ms): "));
  Serial.print(F("Singkat=")); Serial.print(mu_OT_Si, 3);
  Serial.print(F("  Sedang=")); Serial.print(mu_OT_Se, 3);
  Serial.print(F("  Lama="));  Serial.println(mu_OT_L, 3);
  Serial.println(F("  [FIS] --- Firing Strength Agregat ---"));
  Serial.print(F("    alpha_Lancar=")); Serial.print(alpha_Lancar, 3);
  Serial.print(F("  alpha_Padat="));   Serial.print(alpha_Padat,  3);
  Serial.print(F("  alpha_Macet="));   Serial.println(alpha_Macet, 3);

  if (alpha_R7 > 0.01f) {
    if (travel_time2_ms >= travel_time_ms) {
      Serial.print(F("  [FIS] R7b aktif (alpha="));
      Serial.print(alpha_R7, 3);
      Serial.println(F(") -> diduga mogok di titik buta S2-S3 (TT2 dominan)"));
    } else {
      Serial.print(F("  [FIS] R7a aktif (alpha="));
      Serial.print(alpha_R7, 3);
      Serial.println(F(") -> diduga mogok di titik buta S1-S2 (TT1 dominan)"));
    }
  }

  // ── Langkah 4: Defuzzifikasi COG ──
  float cl_crisp = defuzzifyCOG(alpha_Lancar, alpha_Padat, alpha_Macet);
  Serial.print(F("  [FIS] --- COG -> CL* = "));
  Serial.print(cl_crisp, 2);
  Serial.println(F(" ---"));

  return cl_crisp;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [7] FUNGSI AKTUATOR (LED, BUZZER, SERVO)                    ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * Kontrol LED + buzzer non-blocking (berbasis millis).
 *   0 = LANCAR : Hijau ON
 *   1 = PADAT  : Kuning ON, buzzer singkat tiap 800ms
 *   2 = MACET  : Merah berkedip, buzzer alarm tiap 250ms
 */
void controlIndicators(int st) {
  unsigned long now = millis();

  switch (st) {
    case 0:   // LANCAR
      digitalWrite(LED_HIJAU,  HIGH);
      digitalWrite(LED_KUNING, LOW);
      digitalWrite(LED_MERAH,  LOW);
      ledMerahState = false;
      digitalWrite(BUZZER_PIN, LOW);
      break;

    case 1:   // PADAT
      digitalWrite(LED_HIJAU,  LOW);
      digitalWrite(LED_KUNING, HIGH);
      digitalWrite(LED_MERAH,  LOW);
      ledMerahState = false;
      if (now - lastBuzzer >= 800UL) {
        lastBuzzer = now;
        digitalWrite(BUZZER_PIN, HIGH);
        delay(80);
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case 2:   // MACET
      digitalWrite(LED_HIJAU,  LOW);
      digitalWrite(LED_KUNING, LOW);
      if (now - lastBlink >= 300UL) {
        lastBlink     = now;
        ledMerahState = !ledMerahState;
        digitalWrite(LED_MERAH, ledMerahState);
      }
      if (now - lastBuzzer >= 250UL) {
        lastBuzzer = now;
        digitalWrite(BUZZER_PIN, HIGH);
        delay(40);
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;
  }
}

/**
 * State machine palang masuk. Default tertutup.
 * Terbuka jika kendaraan stabil di S1 (debounced) DAN status != MACET.
 * Tetap terbuka selama GATE_CLEARANCE_MS sebelum menutup kembali.
 */
void controlServoMasuk(int st, bool det1_debounced) {
  unsigned long now = millis();

  bool kendaraanMenunggu = det1_debounced;
  bool kapasitasTersedia = (st != 2);

  if (kendaraanMenunggu && kapasitasTersedia) {
    servoMasuk.write(SERVO_BUKA);
    isGateOpen       = true;
    lastGateOpenTime = now;

  } else if (isGateOpen && (now - lastGateOpenTime > GATE_CLEARANCE_MS)) {
    servoMasuk.write(SERVO_TUTUP);
    isGateOpen = false;
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [8] FUNGSI DISPLAY (LCD)                                    ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * Baris 1: Status + tt_fis_input         -> "LANCAR  T:2.8s"
 * Baris 2: Jumlah kendaraan di tol + CL  -> "N:3     CL:12.5"
 * N = tt_count + tt2_count = total kendaraan yang sudah masuk S1
 *     tapi belum keluar S3 (masih berada di dalam area tol).
 */
void updateLCD(int st, float tt_fis_ms, uint8_t vehicle_count, float cl_val) {
  lcd.clear();

  lcd.setCursor(0, 0);
  switch (st) {
    case 0: lcd.print(F("LANCAR  ")); break;
    case 1: lcd.print(F("PADAT   ")); break;
    case 2: lcd.print(F("MACET!! ")); break;
  }
  lcd.print(F("T:"));
  lcd.print(tt_fis_ms / 1000.0f, 1);
  lcd.print(F("s"));

  lcd.setCursor(0, 1);
  lcd.print(F("N:"));
  lcd.print(vehicle_count);
  lcd.print(F("     CL:"));
  lcd.print(cl_val, 1);
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [9] FUNGSI DEBUG SERIAL                                     ║
// ╚══════════════════════════════════════════════════════════════╝

void serialPrintAll() {
  Serial.println(F("================================================"));
  Serial.print(F("[LOOP #")); Serial.print(loopCount); Serial.println(F("]"));

  Serial.println(F("  [Sensor]"));
  Serial.print(F("    S1 (Masuk) : "));
  Serial.print(dist1, 2); Serial.print(F(" cm | Det: ")); Serial.print(detect1);
  Serial.print(F(" | Debounced: ")); Serial.println(detect1_debounced ? F("YA") : F("tidak"));
  Serial.print(F("    S2 (Tengah): "));
  Serial.print(dist2, 2); Serial.print(F(" cm | Det: ")); Serial.println(detect2);
  Serial.print(F("    S3 (Keluar): "));
  Serial.print(dist3, 2); Serial.print(F(" cm | Det: ")); Serial.println(detect3);

  Serial.println(F("  [Microscopic Variables]"));
  unsigned long now_serial = millis();

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

  Serial.print(F("    TT_FIS (MAX) = ")); Serial.print(tt_fis_input, 0);
  Serial.print(F(" ms -> dominan: "));
  if (travel_time2_ms > travel_time_ms) {
    Serial.println(F("TT2 [S2->S3]"));
  } else if (travel_time_ms > 0.0f) {
    Serial.println(F("TT1 [S1->S2]"));
  } else {
    Serial.println(F("keduanya 0 (idle)"));
  }

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

  Serial.println(F("  [Output Sistem]"));
  Serial.print(F("    CL (Congestion Level) = ")); Serial.println(CL, 2);
  Serial.print(F("    STATUS = "));
  switch (status) {
    case 0: Serial.println(F("LANCAR")); break;
    case 1: Serial.println(F("PADAT !")); break;
    case 2: Serial.println(F("MACET (PALANG DITUTUP)")); break;
  }

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


// ╔══════════════════════════════════════════════════════════════╗
// ║  [10] SETUP                                                  ║
// ╚══════════════════════════════════════════════════════════════╝

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("==============================================="));
  Serial.println(F("  SmartToll Density Monitoring -- v3.2.1"));
  Serial.println(F("  ESP32 DevKit | Dual FIFO TT (S1->S3)"));
  Serial.println(F("==============================================="));

  Wire.begin(21, 22);   // SDA, SCL -- default ESP32

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);

  pinMode(LED_HIJAU,  OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH,  OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_HIJAU,  LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH,  LOW);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("SmartToll v3.2.1"));
  lcd.setCursor(0, 1); lcd.print(F("Dual TT | N veh. "));

  servoMasuk.attach(SERVO_MASUK_PIN);
  servoMasuk.write(SERVO_TUTUP);   // default: tertutup

  // ── Animasi startup LED ──
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_HIJAU,  HIGH); delay(150);
    digitalWrite(LED_HIJAU,  LOW);
    digitalWrite(LED_KUNING, HIGH); delay(150);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH,  HIGH); delay(150);
    digitalWrite(LED_MERAH,  LOW);
  }

  // ── Reset ulang seluruh state (defensive) ──
  memset(tt_queue, 0, sizeof(tt_queue));
  tt_head = tt_tail = tt_count = 0;
  travel_time_ms = 0.0f;

  memset(tt2_queue, 0, sizeof(tt2_queue));
  tt2_head = tt2_tail = tt2_count = 0;
  travel_time2_ms = 0.0f;
  tt_fis_input    = 0.0f;

  prev_detect1 = prev_detect2 = prev_detect3 = 0;

  occ1_active = occ2_active = occ3_active = false;
  occupancy_time_ms = 0.0f;

  detect1_stable_since = 0;
  detect1_debounced    = false;

  isGateOpen       = false;
  lastGateOpenTime = 0;

  CL     = 0.0f;
  status = 0;

  delay(800);
  lcd.clear();

  Serial.println(F("  Sistem siap. v3.2.1 ESP32 aktif!"));
  Serial.println(F("    S1 TRIG=13 ECHO=12 | S2 TRIG=14 ECHO=27 | S3 TRIG=26 ECHO=25"));
  Serial.println(F("    Servo=GPIO5 | LED Hijau=18 Kuning=19 Merah=23 | Buzzer=4"));
  Serial.println(F("    LCD I2C SDA=21 SCL=22"));
  Serial.println();
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  [11] LOOP UTAMA                                             ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * Urutan eksekusi tiap siklus loop (~LOOP_DELAY ms):
 *   T1  Baca sensor                  T8  Kontrol LED + Buzzer
 *   T2  Debounce S1                  T9  Kontrol Servo Masuk
 *   T3  Update TT1 + TT2             T10 Update LCD
 *   T4  Update Occupancy Time        T11 Output Serial Monitor
 *   T5  tt_fis_input = MAX(TT1,TT2)  T12 Perbarui prev_detect
 *   T6  FIS Mamdani -> CL            T13 Timing control
 *   T7  Tentukan status
 */
void loop() {
  loopCount++;
  unsigned long loop_start = millis();

  // T1 — Baca semua sensor ultrasonik
  readAllSensors();
  detect1 = detectVehicle(dist1);
  detect2 = detectVehicle(dist2);
  detect3 = detectVehicle(dist3);

  // T2 — Debounce Sensor 1
  unsigned long now_db = millis();
  if (detect1 == 1) {
    if (detect1_stable_since == 0) detect1_stable_since = now_db;
    if ((now_db - detect1_stable_since) >= DEBOUNCE_MS) {
      detect1_debounced = true;
    }
  } else {
    detect1_stable_since = 0;
    detect1_debounced    = false;
  }

  // T3 — Update Travel Time (TT1 + TT2)
  updateTravelTime();

  // T4 — Update Occupancy Time (OT)
  updateOccupancyTime();

  // T5 — tt_fis_input = MAX(TT1, TT2): segmen terburuk masuk FIS
  tt_fis_input = (travel_time_ms > travel_time2_ms)
                 ? travel_time_ms : travel_time2_ms;

  // T6 — Jalankan FIS Mamdani -> CL
  CL = runFuzzyMamdani(tt_fis_input, occupancy_time_ms);

  // T7 — Tentukan status akhir
  if      (CL < CL_BATAS_LANCAR) status = 0;   // LANCAR
  else if (CL < CL_BATAS_PADAT)  status = 1;   // PADAT
  else                            status = 2;   // MACET

  // T8 — Kontrol indikator (LED + buzzer)
  controlIndicators(status);

  // T9 — Kontrol servo masuk (state machine)
  controlServoMasuk(status, detect1_debounced);

  // T10 — Update LCD
  updateLCD(status, tt_fis_input, tt_count + tt2_count, CL);

  // T11 — Output Serial Monitor
  serialPrintAll();

  // T12 — Perbarui prev_detect untuk deteksi rising edge berikutnya
  prev_detect1 = detect1;
  prev_detect2 = detect2;
  prev_detect3 = detect3;

  // T13 — Timing control (kompensasi waktu eksekusi)
  long elapsed = (long)(millis() - loop_start);
  long wait    = (long)LOOP_DELAY - elapsed;
  if (wait > 0) delay((unsigned long)wait);
}
