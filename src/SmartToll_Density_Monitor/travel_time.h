/**
 * travel_time.h — Deklarasi Fungsi Travel Time
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Mengelola buffer FIFO circular untuk menghitung travel time
 * kendaraan antara sensor-sensor.
 */

#ifndef TRAVEL_TIME_H
#define TRAVEL_TIME_H

#include <Arduino.h>
#include "config.h"
#include "globals.h"

// ============================================================
// Operasi Buffer TT1 (S1 → S2)
// ============================================================

/**
 * Mendorong timestamp ke buffer TT1.
 * @param timestamp Waktu millis() saat kendaraan terdeteksi S1
 * @return true jika berhasil, false jika buffer penuh
 */
bool pushTT1(unsigned long timestamp);

/**
 * Mengambil timestamp terlama dari buffer TT1 dan menghitung elapsed time.
 * @param[out] elapsed Hasil selisih waktu (ms)
 * @return true jika berhasil pop, false jika buffer kosong
 */
bool popTT1(unsigned long &elapsed);

/**
 * Melihat elapsed time tanpa menghapus dari buffer (peek).
 * @return Elapsed time (ms) sejak timestamp terlama, atau 0 jika kosong
 */
unsigned long peekTT1();

// ============================================================
// Operasi Buffer TT2 (S2 → S3)
// ============================================================

/**
 * Mendorong timestamp ke buffer TT2.
 * @param timestamp Waktu millis() saat kendaraan terdeteksi S2
 * @return true jika berhasil, false jika buffer penuh
 */
bool pushTT2(unsigned long timestamp);

/**
 * Mengambil timestamp terlama dari buffer TT2 dan menghitung elapsed time.
 * @param[out] elapsed Hasil selisih waktu (ms)
 * @return true jika berhasil pop, false jika buffer kosong
 */
bool popTT2(unsigned long &elapsed);

/**
 * Melihat elapsed time tanpa menghapus dari buffer (peek).
 * @return Elapsed time (ms) sejak timestamp terlama, atau 0 jika kosong
 */
unsigned long peekTT2();

// ============================================================
// Logika Utama Travel Time
// ============================================================

/**
 * Memperbarui travel time berdasarkan rising edge deteksi sensor.
 * - Rising edge S1 → push timestamp ke TT1
 * - Rising edge S2 → pop TT1 (hitung TT1) + push timestamp ke TT2
 * - Rising edge S3 → pop TT2 (hitung TT2)
 * - Idle reset: jika buffer kosong dan semua sensor clean
 */
void updateTravelTime();

/**
 * Mengusir ghost vehicle dari buffer jika timestamp terlalu lama.
 * @param threshold_ms Batas waktu (ms) untuk menganggap entry sebagai ghost
 */
void evictGhosts(unsigned long threshold_ms);

#endif // TRAVEL_TIME_H
