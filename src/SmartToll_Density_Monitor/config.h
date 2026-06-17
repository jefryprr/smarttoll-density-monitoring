#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
//  KONFIGURASI SISTEM — SmartToll Density Monitoring v3.2.1
//  Pin mapping, parameter timing, dan domain variabel fuzzy.
//  Target board: ESP32 DevKit
//
//  Untuk wiring lengkap & alasan tiap nilai, lihat README.md.
// ================================================================

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
// Nilai yang masuk FIS adalah tt_fis_input = MAX(TT1, TT2).
#define TT_MAX_MS  10000UL

// OT (Occupancy Time): durasi sensor terbaca HIGH secara kontinu.
#define OT_MAX_MS  5000UL

// ---------------- Kapasitas Buffer FIFO ----------------
#define TT_QUEUE_CAPACITY   5   // FIFO TT1 (S1→S2)
#define TT2_QUEUE_CAPACITY  5   // FIFO TT2 (S2→S3)

#endif // CONFIG_H
