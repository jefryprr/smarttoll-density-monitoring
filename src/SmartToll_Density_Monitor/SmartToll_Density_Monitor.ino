/**
 * ================================================================
 *  SmartToll Density Monitoring System — v3.2.1 (ESP32 Port)
 *  Tugas Besar Sistem Kendali dan Sensor Aktuator
 * ================================================================
 *
 *  PORT NOTES (Arduino Uno R3 → ESP32 DevKit):
 *  ─────────────────────────────────────────────────────────────
 *  [1] Library Servo  : <Servo.h> diganti <ESP32Servo.h>
 *      Install via Library Manager: "ESP32Servo" by Kevin Harrington
 *  [2] I2C LCD        : Wire.begin(21, 22) ditambahkan di setup()
 *      SDA = GPIO 21 | SCL = GPIO 22 (default ESP32)
 *  [3] Serial baud    : 9600 → 115200 (standar ESP32)
 *  [4] Pin mapping    : Seluruh pin diupdate (lihat DEFINISI PIN)
 *  [5] pulseIn()      : Kompatibel penuh di ESP32 tanpa perubahan
 *  [6] millis()       : Kompatibel penuh di ESP32
 *  [7] F() macro      : Masih valid di ESP32 (opsional, tidak masalah)
 *  [8] memset()       : Kompatibel penuh di ESP32
 *  Semua logika sistem (FIS, FIFO, OT, servo, LCD) tidak diubah
 *  kecuali penambahan TT2 yang didokumentasikan di bawah.
 *
 *  CHANGELOG v3.2.1 (dari v3.2):
 *  ─────────────────────────────────────────────────────────────
 *  [1] updateLCD(): baris 2 ganti dari OT (Occupancy Time)
 *      menjadi N (jumlah kendaraan di dalam area tol).
 *      N = tt_count + tt2_count
 *        = kendaraan di S1→S2 + kendaraan di S2→S3.
 *  [2] Signature updateLCD() diperbarui:
 *      float ot_ms → uint8_t vehicle_count.
 *  [3] Pemanggilan updateLCD() di loop T10 diperbarui:
 *      occupancy_time_ms → tt_count + tt2_count.
 *
 * ================================================================
 *  CHANGELOG v3.2 (dari v3.1):
 * ================================================================
 *
 *  MASALAH v3.1 — Blind Spot S2→S3 Tidak Terdeteksi:
 *  ─────────────────────────────────────────────────────────────
 *  v3.1 hanya melacak TT di segmen S1→S2.
 *  Jika kendaraan MOGOK atau BERHENTI di antara S2 dan S3
 *  (tidak tepat di depan sensor manapun), sistem tidak
 *  mendeteksinya karena:
 *    • TT1 sudah selesai dihitung saat kendaraan melewati S2.
 *    • OT S2 dan S3 = 0 (kendaraan tidak menghalangi sensor).
 *    → FIS output = LANCAR padahal ada kendaraan mogok!
 *
 *  SOLUSI v3.2 — Dual FIFO TT + MAX Strategy:
 *  ─────────────────────────────────────────────────────────────
 *  Ditambahkan FIFO buffer kedua (tt2_queue) untuk melacak
 *  Travel Time di segmen S2→S3 secara paralel.
 *
 *  Alur kerja TT2:
 *    S2 Rising Edge → PUSH ke tt2_queue (mulai stopwatch S2→S3)
 *    S3 Rising Edge → POP  dari tt2_queue (TT2 final dihitung)
 *    Live Tracking  → PEEK tt2_head → TT2 naik real-time
 *    Idle Reset     → tt2_count==0 && S2=0 && S3=0 → TT2=0
 *
 *  Input FIS yang digunakan:
 *    tt_fis_input = MAX(travel_time_ms, travel_time2_ms)
 *    → Segmen paling buruk yang menentukan output FIS.
 *    → Rule R7 (TT=Lama ∧ OT=Singkat → MACET) kini berlaku
 *      untuk KEDUA blind spot: S1→S2 (R7a) DAN S2→S3 (R7b).
 *    → 9 rule FIS tidak perlu diubah sama sekali.
 *
 *  Perubahan kode dari v3.1 → v3.2:
 *    [1] Ditambah: TT2_QUEUE_CAPACITY, tt2_queue[], tt2_head,
 *        tt2_tail, tt2_count, travel_time2_ms, tt_fis_input.
 *    [2] Ditambah: prev_detect3 untuk deteksi Rising Edge S3.
 *    [3] updateTravelTime(): S2 Rising Edge kini JUGA PUSH ke tt2.
 *        S3 Rising Edge: POP dari tt2, hitung TT2 final.
 *        Live TT2 tracking + idle reset TT2 ditambahkan.
 *    [4] loop() T5: tt_fis_input = MAX(TT1,TT2), masuk ke FIS.
 *    [5] loop() T9: updateLCD() menerima tt_fis_input.
 *    [6] loop() T11: prev_detect3 diperbarui.
 *    [7] setup(): inisialisasi TT2 buffer + prev_detect3.
 *    [8] serialPrintAll(): tampilkan TT2 buffer + tt_fis_input.
 *    [9] Catatan R7a/R7b ditambahkan di komentar Rule Base.
 *
 *  Fitur yang DIPERTAHANKAN dari v3.1:
 *    • TT1 FIFO Circular Buffer (S1→S2) — tidak berubah.
 *    • Live TT1 tracking + ghost vehicle eviction (R7a).
 *    • Debounce S1, State Machine Servo, Non-blocking LED/Buzzer.
 *    • FIS Mamdani 9 Rule (TT + OT), Defuzzifikasi COG step-5.
 *    • Semua logika OT (Occupancy Time) tidak berubah.
 *
 * ================================================================
 *  PARADIGMA: MIKROSKOPIS — Discrete Event System (DES)
 * ================================================================
 *  INPUT 1 — WAKTU TEMPUH (Travel Time / TT):
 *    tt_fis_input = MAX( TT1[S1→S2], TT2[S2→S3] )
 *    Segmen mana yang paling buruk, itulah yang masuk ke FIS.
 *    TT pendek → kendaraan melaju wajar.
 *    TT panjang/timeout → kendaraan LAMBAT atau MOGOK di titik buta.
 *
 *  INPUT 2 — DURASI OKUPANSI (Occupancy Time / OT):
 *    Lamanya sensor terbaca HIGH secara kontinu (diambil MAX S1/S2/S3).
 *    OT pendek → kendaraan bergerak melewati sensor.
 *    OT panjang → kendaraan BERHENTI TEPAT di depan sensor.
 *
 *  Komponen Aktif:
 *    - 3x HC-SR04  → Sensor zona masuk (S1), tengah (S2), keluar (S3)
 *    - 1x LCD 16x2 I2C (0x27) — SDA: GPIO21, SCL: GPIO22
 *    - 3x LED (Hijau GPIO18 / Kuning GPIO19 / Merah GPIO23)
 *    - 1x Buzzer aktif (GPIO4)
 *    - 1x Servo SG90 — Palang Masuk (GPIO5 / D5)
 * ================================================================
 */

// ----------------------------------------------------------------
// LIBRARY
// ----------------------------------------------------------------
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>   // ← Ganti dari <Servo.h> untuk ESP32

// ================================================================
// DEFINISI PIN — ESP32 DevKit
// ================================================================

// Sensor Ultrasonik HC-SR04
#define TRIG1  13   // Sensor 1 — Zona Masuk (tepat sebelum palang)
#define ECHO1  12
#define TRIG2  14   // Sensor 2 — Zona Tengah (titik pengukuran TT1 & TT2)
#define ECHO2  27
#define TRIG3  26   // Sensor 3 — Zona Keluar (titik akhir TT2)
#define ECHO3  25

// Servo Motor
#define SERVO_MASUK_PIN  5    // D5 — Palang Masuk

// LED Indikator Status
#define LED_HIJAU   18
#define LED_KUNING  19
#define LED_MERAH   23

// Buzzer Aktif
#define BUZZER_PIN  4

// ================================================================
// KONSTANTA SISTEM
// ================================================================

// --- Parameter Sensor ---
#define BATAS_LEBAR_JALAN  12.0f

// --- Parameter Timing Loop ---
#define LOOP_DELAY    50    // Target durasi satu siklus loop [ms]
#define SENSOR_DELAY  15    // Jeda antar pembacaan sensor [ms]

// --- Parameter Debounce Sensor 1 ---
#define DEBOUNCE_MS  150UL

// --- Parameter Gate Clearance (State Machine Servo) ---
#define GATE_CLEARANCE_MS  200UL

// --- Parameter Servo ---
#define SERVO_BUKA   90    // Sudut terbuka [°]
#define SERVO_TUTUP   0    // Sudut tertutup [°] — DEFAULT

// --- Threshold Klasifikasi Status dari CL ---
#define CL_BATAS_LANCAR  34.0f
#define CL_BATAS_PADAT   67.0f

// ================================================================
// KONSTANTA DOMAIN VARIABEL FUZZY MIKROSKOPIS
// ================================================================

/**
 * WAKTU TEMPUH (Travel Time / TT)
 * Domain: [0, TT_MAX_MS] milidetik
 *   Cepat  : Trapesium [0, 0, 1500, 3000]
 *   Sedang : Segitiga  [2000, 4000, 6000]
 *   Lama   : Trapesium [5000, 8000, 10000, 10000]
 *
 * Catatan v3.2: Domain ini berlaku untuk KEDUA segmen
 *   TT1 (S1→S2) maupun TT2 (S2→S3). Nilai yang masuk ke
 *   FIS adalah MAX(TT1, TT2) = tt_fis_input.
 */
#define TT_MAX_MS  10000UL

/**
 * DURASI OKUPANSI (Occupancy Time / OT)
 * Domain: [0, OT_MAX_MS] milidetik
 *   Singkat : Trapesium [0, 0, 500, 1500]
 *   Sedang  : Segitiga  [1000, 2000, 3000]
 *   Lama    : Trapesium [2500, 4000, 5000, 5000]
 */
#define OT_MAX_MS  5000UL

// ================================================================
// OBJEK GLOBAL
// ================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMasuk;

// ================================================================
// VARIABEL GLOBAL — DATA SENSOR & DETEKSI
// ================================================================

float dist1 = 0.0f;
float dist2 = 0.0f;
float dist3 = 0.0f;

int detect1 = 0;
int detect2 = 0;
int detect3 = 0;

int prev_detect1 = 0;
int prev_detect2 = 0;
int prev_detect3 = 0;   // ← BARU v3.2: untuk deteksi Rising Edge S3

// ================================================================
// VARIABEL GLOBAL — DEBOUNCE SENSOR 1
// ================================================================
unsigned long detect1_stable_since = 0;
bool          detect1_debounced    = false;

// ================================================================
// VARIABEL GLOBAL — CIRCULAR BUFFER TT1: S1→S2 (tidak berubah)
// ================================================================
/**
 * ┌──────────────────────────────────────────────────────────────┐
 * │  TT1 Buffer — mengukur waktu dari S1 ke S2                   │
 * │  PUSH  : S1 Rising Edge                                      │
 * │  POP   : S2 Rising Edge                                      │
 * │  PEEK  : live tracking kendaraan terdepan di segmen S1→S2    │
 * └──────────────────────────────────────────────────────────────┘
 */
#define TT_QUEUE_CAPACITY  5

unsigned long tt_queue[TT_QUEUE_CAPACITY];
uint8_t       tt_head  = 0;
uint8_t       tt_tail  = 0;
uint8_t       tt_count = 0;

float         travel_time_ms = 0.0f;   // TT1 saat ini [ms]

// ================================================================
// VARIABEL GLOBAL — CIRCULAR BUFFER TT2: S2→S3 (BARU v3.2)
// ================================================================
/**
 * ┌──────────────────────────────────────────────────────────────┐
 * │  TT2 Buffer — mengukur waktu dari S2 ke S3                   │
 * │                                                              │
 * │  Kapasitas sama dengan TT1 (5 slot).                         │
 * │  PUSH  : S2 Rising Edge (kendaraan mulai segmen S2→S3)       │
 * │  POP   : S3 Rising Edge (kendaraan selesai segmen S2→S3)     │
 * │  PEEK  : live tracking kendaraan terdepan di segmen S2→S3    │
 * │                                                              │
 * │  Ilustrasi:                                                  │
 * │  [S1]──── TT1 ────[S2]──── TT2 ────[S3]                     │
 * │         (v3.1)           (v3.2 baru)                         │
 * │                                                              │
 * │  Jika ada kendaraan mogok di ANTARA S2 dan S3:               │
 * │    • TT2 terus naik → mencapai TT_MAX_MS                     │
 * │    • OT S2 dan S3 = 0 (tidak menghalangi sensor)             │
 * │    • FIS: TT=Lama ∧ OT=Singkat → R7b aktif → MACET ✓        │
 * └──────────────────────────────────────────────────────────────┘
 */
#define TT2_QUEUE_CAPACITY  5

unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
uint8_t       tt2_head  = 0;
uint8_t       tt2_tail  = 0;
uint8_t       tt2_count = 0;

float         travel_time2_ms = 0.0f;  // TT2 saat ini [ms]

// ================================================================
// VARIABEL GLOBAL — INPUT FIS GABUNGAN (BARU v3.2)
// ================================================================
/**
 * tt_fis_input = MAX(travel_time_ms, travel_time2_ms)
 *
 * Logika: segmen yang paling buruk yang menentukan output FIS.
 * Contoh:
 *   TT1=2000ms, TT2=8500ms → tt_fis_input=8500ms → FIS output MACET
 *   TT1=7000ms, TT2=500ms  → tt_fis_input=7000ms → FIS output MACET
 *   TT1=1200ms, TT2=1500ms → tt_fis_input=1500ms → FIS output LANCAR
 */
float tt_fis_input = 0.0f;

// ================================================================
// VARIABEL GLOBAL — PELACAK DURASI OKUPANSI (OCCUPANCY TIME)
// ================================================================
unsigned long occ1_start_ms = 0;
unsigned long occ2_start_ms = 0;
unsigned long occ3_start_ms = 0;
bool          occ1_active   = false;
bool          occ2_active   = false;
bool          occ3_active   = false;
float         occupancy_time_ms = 0.0f;

// ================================================================
// VARIABEL GLOBAL — OUTPUT FUZZY & STATUS AKHIR
// ================================================================
float CL     = 0.0f;
int   status = 0;

// ================================================================
// VARIABEL GLOBAL — STATE MACHINE SERVO MASUK
// ================================================================
unsigned long lastGateOpenTime = 0;
bool          isGateOpen       = false;

// ================================================================
// VARIABEL GLOBAL — TIMING NON-BLOCKING
// ================================================================
unsigned long lastBlink     = 0;
bool          ledMerahState = false;
unsigned long lastBuzzer    = 0;

// ================================================================
// VARIABEL GLOBAL — MONITORING
// ================================================================
unsigned long loopCount = 0;


// ╔══════════════════════════════════════════════════════════════╗
// ║            BLOK FUNGSI — PEMBACAAN SENSOR                   ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Mengukur jarak satu sensor HC-SR04 (Time-of-Flight).
 * Rumus: d [cm] = t_echo [µs] × 0.01715
 * @return Jarak terukur [cm], atau 30.0 cm jika timeout
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

/**
 * @brief Membaca ketiga sensor secara berurutan (time-multiplexed).
 * SENSOR_DELAY = 15ms mencegah cross-talk antar sensor.
 */
void readAllSensors() {
  dist1 = readUltrasonic(TRIG1, ECHO1);
  delay(SENSOR_DELAY);
  dist2 = readUltrasonic(TRIG2, ECHO2);
  delay(SENSOR_DELAY);
  dist3 = readUltrasonic(TRIG3, ECHO3);
  delay(SENSOR_DELAY);
}

/**
 * @brief Menentukan apakah kendaraan hadir di depan sensor.
 * @return 1 jika kendaraan terdeteksi, 0 jika tidak
 */
int detectVehicle(float d) {
  return (d < BATAS_LEBAR_JALAN) ? 1 : 0;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║  BLOK FUNGSI — PELACAK WAKTU TEMPUH (TRAVEL TIME v3.2)      ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Memperbarui Travel Time TT1 (S1→S2) dan TT2 (S2→S3).
 *
 * ────────────────────────────────────────────────────────────────
 *  ALUR TT1 (S1→S2) — tidak berubah dari v3.1:
 *    PUSH  : S1 Rising Edge → simpan timestamp di tt_queue
 *    POP   : S2 Rising Edge → hitung TT1 final
 *    PEEK  : live tracking TT1 kendaraan terdepan
 *    RESET : tt_count==0 && semua sensor bersih
 *
 *  ALUR TT2 (S2→S3) — BARU v3.2:
 *    PUSH  : S2 Rising Edge → simpan timestamp di tt2_queue
 *            (terjadi BERSAMAAN dengan POP TT1 di S2)
 *    POP   : S3 Rising Edge → hitung TT2 final
 *    PEEK  : live tracking TT2 kendaraan terdepan
 *    RESET : tt2_count==0 && S2==0 && S3==0
 * ────────────────────────────────────────────────────────────────
 *
 *  Ilustrasi kejadian di S2 (satu Rising Edge, dua aksi):
 *
 *    [S1]────────────────[S2]────────────────[S3]
 *          TT1 selesai ──^  ^── TT2 mulai
 *          (POP tt1)           (PUSH tt2)
 *
 *  Hasilnya: tidak ada gap antara TT1 dan TT2.
 *  Seluruh jalur S1→S3 terpantau tanpa blind spot.
 * ────────────────────────────────────────────────────────────────
 */
void updateTravelTime() {
  unsigned long now = millis();

  // ────────────────────────────────────────────────────────────
  // TT1: SEGMEN S1 → S2
  // ────────────────────────────────────────────────────────────

  // ── TT1 FASE 1: Rising Edge S1 → PUSH ke tt_queue ──
  if (detect1 == 1 && prev_detect1 == 0) {
    if (tt_count < TT_QUEUE_CAPACITY) {
      tt_queue[tt_tail] = now;
      tt_tail           = (tt_tail + 1) % TT_QUEUE_CAPACITY;
      tt_count++;
      Serial.print(F("  [TT1] Rising Edge S1 @ "));
      Serial.print(now);
      Serial.print(F(" ms → PUSH. TT1 Antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT1] ⚠ Antrean PENUH ("));
      Serial.print(TT_QUEUE_CAPACITY);
      Serial.println(F(" slot) — S1 Rising Edge diabaikan."));
    }
  }

  // ── TT1 FASE 2 + TT2 FASE 1: Rising Edge S2 ──
  //    AKSI 1 (TT1): POP dari tt_queue → hitung TT1 final
  //    AKSI 2 (TT2): PUSH ke tt2_queue → mulai stopwatch S2→S3
  if (detect2 == 1 && prev_detect2 == 0) {

    // AKSI 1 — Selesaikan TT1
    if (tt_count > 0) {
      unsigned long s1_ts = tt_queue[tt_head];
      tt_head             = (tt_head + 1) % TT_QUEUE_CAPACITY;
      tt_count--;
      float elapsed  = (float)(now - s1_ts);
      travel_time_ms = (elapsed > (float)TT_MAX_MS)
                       ? (float)TT_MAX_MS : elapsed;
      Serial.print(F("  [TT1] Rising Edge S2 → POP. TT1 final = "));
      Serial.print(travel_time_ms, 0);
      Serial.print(F(" ms. Sisa TT1 antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT1] Rising Edge S2 tapi tt1 antrean kosong → diabaikan."));
    }

    // AKSI 2 — Mulai TT2 (v3.2 BARU)
    if (tt2_count < TT2_QUEUE_CAPACITY) {
      tt2_queue[tt2_tail] = now;
      tt2_tail            = (tt2_tail + 1) % TT2_QUEUE_CAPACITY;
      tt2_count++;
      Serial.print(F("  [TT2] Rising Edge S2 → PUSH. TT2 Antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT2] ⚠ Antrean PENUH ("));
      Serial.print(TT2_QUEUE_CAPACITY);
      Serial.println(F(" slot) — S2 PUSH diabaikan."));
    }
  }

  // ── TT2 FASE 2: Rising Edge S3 → POP, hitung TT2 final (v3.2 BARU) ──
  if (detect3 == 1 && prev_detect3 == 0) {
    if (tt2_count > 0) {
      unsigned long s2_ts  = tt2_queue[tt2_head];
      tt2_head             = (tt2_head + 1) % TT2_QUEUE_CAPACITY;
      tt2_count--;
      float elapsed2        = (float)(now - s2_ts);
      travel_time2_ms       = (elapsed2 > (float)TT_MAX_MS)
                              ? (float)TT_MAX_MS : elapsed2;
      Serial.print(F("  [TT2] Rising Edge S3 → POP. TT2 final = "));
      Serial.print(travel_time2_ms, 0);
      Serial.print(F(" ms. Sisa TT2 antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT2] Rising Edge S3 tapi tt2 antrean kosong → diabaikan."));
    }
  }

  // ── TT1 FASE 3: Live Tracking — PEEK kendaraan terdepan di S1→S2 ──
  // Jika kendaraan mogok antara S1 dan S2, TT1 terus naik → R7a aktif.
  if (tt_count > 0) {
    float elapsed  = (float)(now - tt_queue[tt_head]);
    travel_time_ms = (elapsed > (float)TT_MAX_MS)
                     ? (float)TT_MAX_MS : elapsed;
  }

  // ── TT2 FASE 3: Live Tracking — PEEK kendaraan terdepan di S2→S3 (v3.2 BARU) ──
  // Jika kendaraan mogok antara S2 dan S3, TT2 terus naik → R7b aktif.
  if (tt2_count > 0) {
    float elapsed2   = (float)(now - tt2_queue[tt2_head]);
    travel_time2_ms  = (elapsed2 > (float)TT_MAX_MS)
                       ? (float)TT_MAX_MS : elapsed2;
  }

  // ── TT1 FASE 4: Idle Reset ──
  if (tt_count == 0 && detect1 == 0 && detect2 == 0 && detect3 == 0) {
    travel_time_ms = 0.0f;
  }

  // ── TT2 FASE 4: Idle Reset (v3.2 BARU) ──
  // Reset TT2 jika tidak ada kendaraan aktif di segmen S2→S3.
  if (tt2_count == 0 && detect2 == 0 && detect3 == 0) {
    travel_time2_ms = 0.0f;
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║   BLOK FUNGSI — PELACAK DURASI OKUPANSI (OCCUPANCY TIME)    ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Memperbarui nilai Occupancy Time (OT) setiap siklus loop.
 * Nilai OT untuk FIS = MAX(occ1, occ2, occ3) → kondisi terparah.
 */
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


// ╔══════════════════════════════════════════════════════════════╗
// ║       BLOK FUNGSI — MEMBERSHIP FUNCTIONS (FUZZY)            ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Triangular Membership Function — bentuk segitiga.
 *   μ(x) = 0               , x ≤ a atau x ≥ c
 *   μ(x) = (x-a)/(b-a)    , a < x ≤ b
 *   μ(x) = (c-x)/(c-b)    , b < x < c
 */
float triangularMF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x <= b)           return (x - a) / (b - a);
  return                       (c - x) / (c - b);
}

/**
 * @brief Trapezoidal Membership Function — bentuk trapesium.
 *   μ(x) = 0               , x < a atau x > d
 *   μ(x) = (x-a)/(b-a)    , a ≤ x < b  (sisi naik)
 *   μ(x) = 1               , b ≤ x ≤ c  (plateau)
 *   μ(x) = (d-x)/(d-c)    , c < x ≤ d  (sisi turun)
 */
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

// ─── MF INPUT 1: TT (Waktu Tempuh) — domain [0, 10000] ms ───
// Berlaku untuk tt_fis_input = MAX(TT1, TT2)
float mf_TT_Cepat(float x)  { return trapezoidalMF(x,    0.0f,    0.0f,  1500.0f,  3000.0f); }
float mf_TT_Sedang(float x) { return triangularMF (x, 2000.0f, 4000.0f,  6000.0f); }
float mf_TT_Lama(float x)   { return trapezoidalMF(x, 5000.0f, 8000.0f, 10000.0f, 10000.0f); }

// ─── MF INPUT 2: OT (Durasi Okupansi) — domain [0, 5000] ms ───
float mf_OT_Singkat(float x) { return trapezoidalMF(x,    0.0f,    0.0f,  500.0f, 1500.0f); }
float mf_OT_Sedang(float x)  { return triangularMF (x, 1000.0f, 2000.0f, 3000.0f); }
float mf_OT_Lama(float x)    { return trapezoidalMF(x, 2500.0f, 4000.0f, 5000.0f, 5000.0f); }

// ─── MF OUTPUT: CL (Congestion Level) — domain [0, 100] ───
float mf_CL_Lancar(float z) { return trapezoidalMF(z,  0.0f,  0.0f, 10.0f, 40.0f); }
float mf_CL_Padat(float z)  { return triangularMF (z, 20.0f, 50.0f, 80.0f); }
float mf_CL_Macet(float z)  { return trapezoidalMF(z, 60.0f, 80.0f, 100.0f, 100.0f); }


// ╔══════════════════════════════════════════════════════════════╗
// ║         BLOK FUNGSI — DEFUZZIFIKASI COG (step=5)            ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Defuzzifikasi Centre of Gravity diskrit, step=5 (21 iterasi).
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

    // Implikasi Mamdani (Clipping — MIN)
    float clipped_L = (a_lancar < mu_L) ? a_lancar : mu_L;
    float clipped_P = (a_padat  < mu_P) ? a_padat  : mu_P;
    float clipped_M = (a_macet  < mu_M) ? a_macet  : mu_M;

    // Agregasi output (MAX)
    float agg = clipped_L;
    if (clipped_P > agg) agg = clipped_P;
    if (clipped_M > agg) agg = clipped_M;

    numerator   += fz * agg;
    denominator += agg;
  }

  if (denominator < 0.001f) return 0.0f;
  return numerator / denominator;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║    BLOK FUNGSI — FIS MAMDANI MIKROSKOPIS (Inti Sistem)      ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Pipeline lengkap FIS Mamdani: [tt_fis_input, OT] → CL*.
 *
 * ══════════════════════════════════════════════════════════════════
 * RULE BASE MAMDANI (9 Rules — AND=MIN, Agregasi Output=MAX)
 *
 * INPUT TT = tt_fis_input = MAX(TT1[S1→S2], TT2[S2→S3])
 * ══════════════════════════════════════════════════════════════════
 * R1: TT=Cepat  ∧ OT=Singkat → CL=Lancar  (kendaraan melaju wajar)
 * R2: TT=Cepat  ∧ OT=Sedang  → CL=Lancar  (kendaraan panjang/truk)
 * R3: TT=Cepat  ∧ OT=Lama   → CL=Padat   (berhenti mendadak)
 * R4: TT=Sedang ∧ OT=Singkat → CL=Lancar  (kecepatan moderat)
 * R5: TT=Sedang ∧ OT=Sedang  → CL=Padat   (mulai menumpuk)
 * R6: TT=Sedang ∧ OT=Lama   → CL=Macet   (hampir berhenti)
 * R7: TT=Lama   ∧ OT=Singkat → CL=Macet  ★ DETEKSI BLIND SPOT ★
 *       R7a: TT1 dominan → kendaraan mogok di S1→S2 (seperti v3.1)
 *       R7b: TT2 dominan → kendaraan mogok di S2→S3 (BARU v3.2)
 *       Keduanya terpantau otomatis via MAX(TT1, TT2).
 * R8: TT=Lama   ∧ OT=Sedang  → CL=Macet   (antrian terstruktur)
 * R9: TT=Lama   ∧ OT=Lama   → CL=Macet   (mogok di depan sensor)
 * ══════════════════════════════════════════════════════════════════
 */
float runFuzzyMamdani(float tt_val, float ot_val) {

  // ════════════════════════════════════════════════════════════
  // LANGKAH 1: FUZZIFIKASI
  // ════════════════════════════════════════════════════════════
  float mu_TT_C  = mf_TT_Cepat(tt_val);
  float mu_TT_S  = mf_TT_Sedang(tt_val);
  float mu_TT_L  = mf_TT_Lama(tt_val);

  float mu_OT_Si = mf_OT_Singkat(ot_val);
  float mu_OT_Se = mf_OT_Sedang(ot_val);
  float mu_OT_L  = mf_OT_Lama(ot_val);

  // ════════════════════════════════════════════════════════════
  // LANGKAH 2: EVALUASI RULE — Firing Strength (α = MIN)
  // ════════════════════════════════════════════════════════════
  float alpha_R1 = (mu_TT_C < mu_OT_Si) ? mu_TT_C : mu_OT_Si;
  float alpha_R2 = (mu_TT_C < mu_OT_Se) ? mu_TT_C : mu_OT_Se;
  float alpha_R3 = (mu_TT_C < mu_OT_L)  ? mu_TT_C : mu_OT_L;
  float alpha_R4 = (mu_TT_S < mu_OT_Si) ? mu_TT_S : mu_OT_Si;
  float alpha_R5 = (mu_TT_S < mu_OT_Se) ? mu_TT_S : mu_OT_Se;
  float alpha_R6 = (mu_TT_S < mu_OT_L)  ? mu_TT_S : mu_OT_L;
  float alpha_R7 = (mu_TT_L < mu_OT_Si) ? mu_TT_L : mu_OT_Si;
  float alpha_R8 = (mu_TT_L < mu_OT_Se) ? mu_TT_L : mu_OT_Se;
  float alpha_R9 = (mu_TT_L < mu_OT_L)  ? mu_TT_L : mu_OT_L;

  // ════════════════════════════════════════════════════════════
  // LANGKAH 3: AGREGASI PER KELAS OUTPUT (MAX)
  // ════════════════════════════════════════════════════════════
  float alpha_Lancar = alpha_R1;
  if (alpha_R2 > alpha_Lancar) alpha_Lancar = alpha_R2;
  if (alpha_R4 > alpha_Lancar) alpha_Lancar = alpha_R4;

  float alpha_Padat = (alpha_R3 > alpha_R5) ? alpha_R3 : alpha_R5;

  float alpha_Macet = alpha_R6;
  if (alpha_R7 > alpha_Macet) alpha_Macet = alpha_R7;
  if (alpha_R8 > alpha_Macet) alpha_Macet = alpha_R8;
  if (alpha_R9 > alpha_Macet) alpha_Macet = alpha_R9;

  // ─── Log detail FIS ke Serial Monitor ───
  Serial.println(F("  [FIS] ─── Fuzzifikasi ───"));
  Serial.print(F("    TT_FIS(")); Serial.print(tt_val, 0); Serial.print(F(" ms): "));
  Serial.print(F("Cepat="));  Serial.print(mu_TT_C, 3);
  Serial.print(F("  Sedang=")); Serial.print(mu_TT_S, 3);
  Serial.print(F("  Lama="));  Serial.println(mu_TT_L, 3);
  Serial.print(F("    OT(")); Serial.print(ot_val, 0); Serial.print(F(" ms): "));
  Serial.print(F("Singkat=")); Serial.print(mu_OT_Si, 3);
  Serial.print(F("  Sedang=")); Serial.print(mu_OT_Se, 3);
  Serial.print(F("  Lama="));  Serial.println(mu_OT_L, 3);
  Serial.println(F("  [FIS] ─── Firing Strength Agregat ───"));
  Serial.print(F("    α_Lancar=")); Serial.print(alpha_Lancar, 3);
  Serial.print(F("  α_Padat="));   Serial.print(alpha_Padat,  3);
  Serial.print(F("  α_Macet="));   Serial.println(alpha_Macet, 3);
  if (alpha_R7 > 0.01f) {
    // Identifikasi blind spot mana yang aktif
    if (travel_time2_ms >= travel_time_ms) {
      Serial.print(F("  [FIS] ⚠ R7b aktif (α="));
      Serial.print(alpha_R7, 3);
      Serial.println(F(") → Diduga mogok di titik buta S2–S3! (TT2 dominan)"));
    } else {
      Serial.print(F("  [FIS] ⚠ R7a aktif (α="));
      Serial.print(alpha_R7, 3);
      Serial.println(F(") → Diduga mogok di titik buta S1–S2! (TT1 dominan)"));
    }
  }

  // ════════════════════════════════════════════════════════════
  // LANGKAH 4: DEFUZZIFIKASI COG
  // ════════════════════════════════════════════════════════════
  float cl_crisp = defuzzifyCOG(alpha_Lancar, alpha_Padat, alpha_Macet);
  Serial.print(F("  [FIS] ─── COG → CL* = "));
  Serial.print(cl_crisp, 2);
  Serial.println(F(" ───"));

  return cl_crisp;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║         BLOK FUNGSI — KONTROL INDIKATOR (LED + BUZZER)      ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Kontrol LED dan buzzer secara non-blocking (millis).
 * LANCAR : LED Hijau ON  | Buzzer OFF
 * PADAT  : LED Kuning ON | Buzzer singkat tiap 800ms
 * MACET  : LED Merah KEDIP | Buzzer alarm tiap 250ms
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


// ╔══════════════════════════════════════════════════════════════╗
// ║      BLOK FUNGSI — ADMISSION CONTROL (SERVO MASUK)          ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Kontrol palang masuk gerbang tol (State Machine Servo).
 * DEFAULT: Servo TERTUTUP (0°).
 * BUKA: det1_debounced=true DAN status≠MACET.
 * CLEARANCE 2 detik sebelum menutup kembali.
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
// ║            BLOK FUNGSI — UPDATE LCD (v3.2)                  ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Tampilan LCD 16×2 v3.2.
 *
 * Layout:
 * ╔════════════════╗
 * ║LANCAR  T: 2.8s ║  ← Status + tt_fis_input (MAX TT1,TT2)
 * ║N:3     CL:12.5 ║  ← Jumlah kendaraan di dalam tol + CL
 * ╚════════════════╝
 *
 * Catatan:
 *   T = tt_fis_input = MAX(TT1, TT2), segmen terburuk ke FIS.
 *   N = tt_count + tt2_count = total kendaraan yang sudah masuk
 *       S1 tapi belum keluar S3 (masih berada di dalam area tol).
 */
void updateLCD(int st, float tt_fis_ms, uint8_t vehicle_count, float cl_val) {
  lcd.clear();

  // ── Baris 1: Status + TT FIS Input ──
  lcd.setCursor(0, 0);
  switch (st) {
    case 0: lcd.print(F("LANCAR  ")); break;
    case 1: lcd.print(F("PADAT   ")); break;
    case 2: lcd.print(F("MACET!! ")); break;
  }
  lcd.print(F("T:"));
  lcd.print(tt_fis_ms / 1000.0f, 1);
  lcd.print(F("s"));

  // ── Baris 2: Jumlah kendaraan + CL ──
  // N = kendaraan yang sedang ada di dalam area tol
  //   = tt_count (S1→S2) + tt2_count (S2→S3)
  lcd.setCursor(0, 1);
  lcd.print(F("N:"));
  lcd.print(vehicle_count);
  lcd.print(F("     CL:"));
  lcd.print(cl_val, 1);
}


// ╔══════════════════════════════════════════════════════════════╗
// ║       BLOK FUNGSI — OUTPUT SERIAL MONITOR (v3.2)            ║
// ╚══════════════════════════════════════════════════════════════╝

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
  Serial.print(F("    TT1 [S1→S2] = ")); Serial.print(travel_time_ms, 0);
  Serial.print(F(" ms"));
  if (tt_count > 0) {
    Serial.print(F(" | ⏱ LIVE — "));
    Serial.print(tt_count);
    Serial.println(F(" kendaraan di jalur S1→S2"));
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

  // ── TT2 (S2→S3) — BARU v3.2 ──
  Serial.print(F("    TT2 [S2→S3] = ")); Serial.print(travel_time2_ms, 0);
  Serial.print(F(" ms"));
  if (tt2_count > 0) {
    Serial.print(F(" | ⏱ LIVE — "));
    Serial.print(tt2_count);
    Serial.println(F(" kendaraan di jalur S2→S3"));
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
  Serial.print(F(" ms → dominan: "));
  if (travel_time2_ms > travel_time_ms) {
    Serial.println(F("TT2 [S2→S3]"));
  } else if (travel_time_ms > 0.0f) {
    Serial.println(F("TT1 [S1→S2]"));
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
      Serial.println(F("BUKA  [Kendaraan stabil di S1 & tidak macet → izin masuk]"));
    } else {
      unsigned long elapsed_c = millis() - lastGateOpenTime;
      unsigned long sisa = (GATE_CLEARANCE_MS > elapsed_c)
                           ? (GATE_CLEARANCE_MS - elapsed_c) : 0UL;
      Serial.print(F("BUKA  [Clearance aktif, sisa ~"));
      Serial.print(sisa); Serial.println(F(" ms sebelum tutup]"));
    }
  } else {
    if (status == 2 && detect1_debounced) {
      Serial.println(F("TUTUP [MACET → kendaraan baru DIBLOKIR]"));
    } else {
      Serial.println(F("TUTUP [Tidak ada kendaraan stabil / kondisi aman]"));
    }
  }

  Serial.println(F("================================================"));
  Serial.println();
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                          SETUP                              ║
// ╚══════════════════════════════════════════════════════════════╝

void setup() {
  // ── Serial Monitor (115200 — standar ESP32) ──
  Serial.begin(115200);
  Serial.println(F(""));
  Serial.println(F("╔══════════════════════════════════════════╗"));
  Serial.println(F("║  SmartToll Density Monitoring — v3.2.1  ║"));
  Serial.println(F("║  ESP32 DevKit | Dual FIFO TT (S1→S3)    ║"));
  Serial.println(F("║  Sistem Kendali & Sensor Aktuator        ║"));
  Serial.println(F("╚══════════════════════════════════════════╝"));
  Serial.println(F("  Inisialisasi sistem v3.2 (ESP32)..."));

  // ── I2C untuk LCD (SDA=21, SCL=22 — default ESP32) ──
  Wire.begin(21, 22);

  // ── Setup pin mode sensor ──
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);

  // ── Setup pin mode aktuator ──
  pinMode(LED_HIJAU,  OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH,  OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // ── Kondisi awal: semua indikator mati ──
  digitalWrite(LED_HIJAU,  LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH,  LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ── Inisialisasi LCD ──
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("SmartToll v3.2.1"));
  lcd.setCursor(0, 1); lcd.print(F("Dual TT | N veh. "));

  // ── Inisialisasi Servo (ESP32Servo) — DEFAULT: TERTUTUP ──
  servoMasuk.attach(SERVO_MASUK_PIN);
  servoMasuk.write(SERVO_TUTUP);
  Serial.println(F("  Servo Masuk (GPIO5/D5) → TUTUP (0°) [default]"));

  // ── Animasi startup LED ──
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_HIJAU,  HIGH); delay(150);
    digitalWrite(LED_HIJAU,  LOW);
    digitalWrite(LED_KUNING, HIGH); delay(150);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH,  HIGH); delay(150);
    digitalWrite(LED_MERAH,  LOW);
  }

  // ── Inisialisasi TT1 Circular Buffer (S1→S2) ──
  memset(tt_queue, 0, sizeof(tt_queue));
  tt_head        = 0;
  tt_tail        = 0;
  tt_count       = 0;
  travel_time_ms = 0.0f;
  Serial.print(F("  TT1 Buffer (S1→S2) → RESET ("));
  Serial.print(TT_QUEUE_CAPACITY);
  Serial.println(F(" slot FIFO)"));

  // ── Inisialisasi TT2 Circular Buffer (S2→S3) — BARU v3.2 ──
  memset(tt2_queue, 0, sizeof(tt2_queue));
  tt2_head         = 0;
  tt2_tail         = 0;
  tt2_count        = 0;
  travel_time2_ms  = 0.0f;
  tt_fis_input     = 0.0f;
  Serial.print(F("  TT2 Buffer (S2→S3) → RESET ("));
  Serial.print(TT2_QUEUE_CAPACITY);
  Serial.println(F(" slot FIFO) [v3.2 baru]"));

  // ── Inisialisasi prev_detect ──
  prev_detect1   = 0;
  prev_detect2   = 0;
  prev_detect3   = 0;   // ← BARU v3.2

  // ── Inisialisasi variabel Occupancy Time ──
  occ1_active = false;
  occ2_active = false;
  occ3_active = false;
  occupancy_time_ms = 0.0f;
  Serial.println(F("  OT Tracker → RESET (semua sensor bersih)"));

  // ── Inisialisasi Debounce S1 ──
  detect1_stable_since = 0;
  detect1_debounced    = false;

  // ── Inisialisasi State Machine Servo ──
  isGateOpen       = false;
  lastGateOpenTime = 0;

  // ── Inisialisasi FIS output ──
  CL     = 0.0f;
  status = 0;

  delay(800);
  lcd.clear();

  Serial.println(F("  Sistem siap. v3.2 ESP32 aktif!"));
  Serial.println(F("  ────────────────────────────────────────────────────"));
  Serial.println(F("  PIN MAP ESP32:"));
  Serial.println(F("    S1 TRIG=13  ECHO=12  |  S2 TRIG=14  ECHO=27"));
  Serial.println(F("    S3 TRIG=26  ECHO=25  |  Servo = GPIO5 (D5)"));
  Serial.println(F("    LED: Hijau=18  Kuning=19  Merah=23  Buzzer=4"));
  Serial.println(F("    LCD I2C: SDA=21  SCL=22"));
  Serial.println(F("  ────────────────────────────────────────────────────"));
  Serial.println(F("  INFO PARADIGMA v3.2:"));
  Serial.println(F("    TT1 (S1→S2) : FIFO — deteksi mogok di blind spot S1-S2 (R7a)"));
  Serial.println(F("    TT2 (S2→S3) : FIFO — deteksi mogok di blind spot S2-S3 (R7b) [BARU]"));
  Serial.println(F("    TT_FIS      : MAX(TT1, TT2) → segmen terburuk masuk FIS"));
  Serial.println(F("    OT          : Deteksi kendaraan diam tepat di depan sensor"));
  Serial.println(F("    9 Rule FIS  : Tidak berubah — R7 otomatis cover kedua blind spot"));
  Serial.println(F("  ────────────────────────────────────────────────────"));
  Serial.println();
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                       LOOP UTAMA                            ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Loop utama sistem — berulang setiap ~LOOP_DELAY ms.
 *
 * Urutan eksekusi:
 * ┌─────────────────────────────────────────────────────────────┐
 * │ T1:  Baca sensor                                            │
 * │ T2:  Debounce S1                                            │
 * │ T3:  Update TT1 (S1→S2) + TT2 (S2→S3) — Dual FIFO        │
 * │ T4:  Update Occupancy Time (OT) — Continuous Tracking       │
 * │ T5:  Hitung tt_fis_input = MAX(TT1, TT2)                   │
 * │ T6:  Jalankan FIS Mamdani(tt_fis_input, OT) → CL           │
 * │ T7:  Tentukan Status (LANCAR/PADAT/MACET)                   │
 * │ T8:  Kontrol LED + Buzzer (non-blocking)                    │
 * │ T9:  Kontrol Servo Masuk (State Machine)                    │
 * │ T10: Update LCD (tampilkan tt_fis_input)                    │
 * │ T11: Output Serial Monitor                                   │
 * │ T12: Perbarui prev_detect (S1, S2, S3) untuk rising edge    │
 * │ T13: Timing control (kompensasi waktu eksekusi)             │
 * └─────────────────────────────────────────────────────────────┘
 */
void loop() {
  loopCount++;
  unsigned long loop_start = millis();

  // ════════════════════════════════════════
  // T1 — BACA SEMUA SENSOR ULTRASONIK
  // ════════════════════════════════════════
  readAllSensors();
  detect1 = detectVehicle(dist1);
  detect2 = detectVehicle(dist2);
  detect3 = detectVehicle(dist3);

  // ════════════════════════════════════════
  // T2 — DEBOUNCE SENSOR 1
  // ════════════════════════════════════════
  {
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
  }

  // ════════════════════════════════════════
  // T3 — UPDATE TRAVEL TIME (TT1 + TT2)
  // ════════════════════════════════════════
  updateTravelTime();

  // ════════════════════════════════════════
  // T4 — UPDATE OCCUPANCY TIME (OT)
  // ════════════════════════════════════════
  updateOccupancyTime();

  // ════════════════════════════════════════
  // T5 — HITUNG tt_fis_input = MAX(TT1, TT2)
  // ════════════════════════════════════════
  // Segmen mana yang paling buruk, itulah yang masuk ke FIS.
  // Ini memastikan blind spot S1→S2 (R7a) maupun S2→S3 (R7b)
  // sama-sama terpantau tanpa mengubah rule base FIS.
  tt_fis_input = (travel_time_ms > travel_time2_ms)
                 ? travel_time_ms : travel_time2_ms;

  // ════════════════════════════════════════
  // T6 — JALANKAN FIS MAMDANI → CL
  // ════════════════════════════════════════
  CL = runFuzzyMamdani(tt_fis_input, occupancy_time_ms);

  // ════════════════════════════════════════
  // T7 — TENTUKAN STATUS AKHIR
  // ════════════════════════════════════════
  if      (CL < CL_BATAS_LANCAR) status = 0;   // LANCAR
  else if (CL < CL_BATAS_PADAT)  status = 1;   // PADAT
  else                            status = 2;   // MACET

  // ════════════════════════════════════════
  // T8 — KONTROL INDIKATOR (LED + BUZZER)
  // ════════════════════════════════════════
  controlIndicators(status);

  // ════════════════════════════════════════
  // T9 — KONTROL SERVO MASUK (STATE MACHINE)
  // ════════════════════════════════════════
  controlServoMasuk(status, detect1_debounced);

  // ════════════════════════════════════════
  // T10 — UPDATE TAMPILAN LCD
  // ════════════════════════════════════════
  // N = tt_count + tt2_count = total kendaraan di dalam area tol
  //   tt_count  : kendaraan di segmen S1→S2 (belum sampai S2)
  //   tt2_count : kendaraan di segmen S2→S3 (belum sampai S3)
  updateLCD(status, tt_fis_input, tt_count + tt2_count, CL);

  // ════════════════════════════════════════
  // T11 — OUTPUT SERIAL MONITOR
  // ════════════════════════════════════════
  serialPrintAll();

  // ════════════════════════════════════════
  // T12 — PERBARUI NILAI PREV_DETECT
  // ════════════════════════════════════════
  prev_detect1 = detect1;
  prev_detect2 = detect2;
  prev_detect3 = detect3;   // ← BARU v3.2: deteksi Rising Edge S3

  // ════════════════════════════════════════
  // T13 — TIMING CONTROL
  // ════════════════════════════════════════
  long elapsed = (long)(millis() - loop_start);
  long wait    = (long)LOOP_DELAY - elapsed;
  if (wait > 0) delay((unsigned long)wait);
}
