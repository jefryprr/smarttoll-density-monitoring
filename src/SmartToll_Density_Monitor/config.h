/**
 * config.h — Konfigurasi Pin dan Konstanta Sistem
 * SmartToll Density Monitoring System v3.2.1
 * 
 * File ini berisi semua definisi pin, konstanta waktu, dan parameter sistem.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// Konstanta Fisik dan Waktu
// ============================================================
#define BATAS_LEBAR_JALAN     12.0f    // cm — ambang deteksi kendaraan
#define LOOP_DELAY            50       // ms — delay utama loop
#define SENSOR_DELAY          15       // ms — delay antar pembacaan sensor
#define DEBOUNCE_MS           150      // ms — waktu debounce sensor S1
#define GATE_CLEARANCE_MS     200      // ms — waktu jeda buka-tutup palang

// ============================================================
// Kapasitas Antrian Travel Time
// ============================================================
#define TT_QUEUE_CAPACITY     5        // kapasitas buffer TT1 (S1→S2)
#define TT2_QUEUE_CAPACITY    5        // kapasitas buffer TT2 (S2→S3)

// ============================================================
// Batas Normalisasi FIS
// ============================================================
#define TT_MAX_MS             10000.0f // ms — batas atas normalisasi TT
#define OT_MAX_MS             5000.0f  // ms — batas atas normalisasi OT

// ============================================================
// Batas Klasifikasi Congestion Level (CL)
// ============================================================
#define CL_BATAS_LANCAR       34.0f    // CL ≤ 34 → Lancar
#define CL_BATAS_PADAT        67.0f    // CL ≤ 67 → Padat, else Macet

// ============================================================
// Sudut Servo
// ============================================================
#define SERVO_BUKA            90       // derajat — posisi buka palang
#define SERVO_TUTUP           0        // derajat — posisi tutup palang

// ============================================================
// Pin Mapping — Sensor Ultrasonik
// ============================================================
#define S1_TRIG               13       // Trigger sensor S1
#define S1_ECHO               12       // Echo sensor S1
#define S2_TRIG               14       // Trigger sensor S2
#define S2_ECHO               27       // Echo sensor S2
#define S3_TRIG               26       // Trigger sensor S3
#define S3_ECHO               25       // Echo sensor S3

// ============================================================
// Pin Mapping — Aktuator
// ============================================================
#define SERVO_PIN             5        // Pin sinyal servo
#define LED_HIJAU             18       // LED hijau (Lancar)
#define LED_KUNING            19       // LED kuning (Padat)
#define LED_MERAH             23       // LED merah (Macet)
#define BUZZER_PIN            4        // Pin buzzer

// ============================================================
// Konfigurasi LCD I2C
// ============================================================
#define LCD_ADDR              0x27     // Alamat I2C LCD
#define LCD_COLS              16       // Jumlah kolom LCD
#define LCD_ROWS              2        // Jumlah baris LCD

// ============================================================
// Status Enum — Klasifikasi Kemacetan
// ============================================================
enum StatusKemacetan {
    LANCAR = 0,
    PADAT  = 1,
    MACET  = 2
};

#endif // CONFIG_H
