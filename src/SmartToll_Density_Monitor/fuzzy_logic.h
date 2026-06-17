/**
 * fuzzy_logic.h — Deklarasi Fungsi Fuzzy Inference System
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Implementasi FIS Mamdani dengan:
 * - Input: Travel Time (TT) dan Occupancy Time (OT)
 * - Output: Congestion Level (CL)
 * - 9 aturan fuzzy (R1–R9)
 * - Defuzzifikasi: Center of Gravity (COG)
 */

#ifndef FUZZY_LOGIC_H
#define FUZZY_LOGIC_H

#include <Arduino.h>
#include "config.h"
#include "globals.h"

// ============================================================
// Fungsi Keanggotaan Helper
// ============================================================

/**
 * Trapezoidal membership function.
 * @param x  Input value
 * @param a  Kiri bawah
 * @param b  Kiri atas
 * @param c  Kanan atas
 * @param d  Kanan bawah
 * @return Derajat keanggotaan [0.0, 1.0]
 */
float trapmf(float x, float a, float b, float c, float d);

/**
 * Triangular membership function.
 * @param x  Input value
 * @param a  Kiri
 * @param b  Puncak
 * @param c  Kanan
 * @return Derajat keanggotaan [0.0, 1.0]
 */
float trimf(float x, float a, float b, float c);

// ============================================================
// Fungsi Keanggotaan Input TT (Travel Time)
// ============================================================

/**
 * Membership function TT: Cepat [0, 0, 1500, 3000]
 */
float tt_cepat(float x);

/**
 * Membership function TT: Sedang [2000, 4000, 6000]
 */
float tt_sedang(float x);

/**
 * Membership function TT: Lama [5000, 8000, 10000, 10000]
 */
float tt_lama(float x);

// ============================================================
// Fungsi Keanggotaan Input OT (Occupancy Time)
// ============================================================

/**
 * Membership function OT: Singkat [0, 0, 500, 1500]
 */
float ot_singkat(float x);

/**
 * Membership function OT: Sedang [1000, 2000, 3000]
 */
float ot_sedang(float x);

/**
 * Membership function OT: Lama [2500, 4000, 5000, 5000]
 */
float ot_lama(float x);

// ============================================================
// Fungsi Keanggotaan Output CL (Congestion Level)
// ============================================================

/**
 * Membership function CL: Lancar [0, 0, 10, 40]
 */
float cl_lancar(float x);

/**
 * Membership function CL: Padat [20, 50, 80]
 */
float cl_padat(float x);

/**
 * Membership function CL: Macet [60, 80, 100, 100]
 */
float cl_macet(float x);

// ============================================================
// FIS Utama
// ============================================================

/**
 * Menjalankan Fuzzy Inference System (Mamdani).
 * @param tt  Input Travel Time (ms)
 * @param ot  Input Occupancy Time (ms)
 * @return    Congestion Level [0.0, 100.0]
 */
float runFIS(float tt, float ot);

/**
 * Mengklasifikasikan Congestion Level menjadi status.
 * @param cl  Congestion Level
 * @return    LANCAR, PADAT, atau MACET
 */
StatusKemacetan classifyStatus(float cl);

#endif // FUZZY_LOGIC_H
