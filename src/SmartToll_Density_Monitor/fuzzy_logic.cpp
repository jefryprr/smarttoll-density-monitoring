/**
 * fuzzy_logic.cpp — Implementasi Fuzzy Inference System (Mamdani)
 * SmartToll Density Monitoring System v3.2.1
 * 
 * 9 aturan fuzzy:
 * R1: TT Cepat  AND OT Singkat → CL Lancar
 * R2: TT Cepat  AND OT Sedang  → CL Lancar
 * R3: TT Cepat  AND OT Lama    → CL Padat
 * R4: TT Sedang AND OT Singkat → CL Lancar
 * R5: TT Sedang AND OT Sedang  → CL Padat
 * R6: TT Sedang AND OT Lama    → CL Macet
 * R7: TT Lama   AND OT Singkat → CL Padat  (blind spot — gunakan MAX TT)
 * R8: TT Lama   AND OT Sedang  → CL Macet
 * R9: TT Lama   AND OT Lama    → CL Macet
 */

#include "fuzzy_logic.h"

// Resolusi defuzzifikasi COG
#define COG_STEP    5
#define COG_MIN     0.0f
#define COG_MAX     100.0f

// ============================================================
// Fungsi Keanggotaan Helper
// ============================================================

float trapmf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x > a && x < b) return (x - a) / (b - a);
    // x > c && x < d
    return (d - x) / (d - c);
}

float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    // x > b
    return (c - x) / (c - b);
}

// ============================================================
// Fungsi Keanggotaan Input TT
// ============================================================

float tt_cepat(float x) {
    // Trapmf [0, 0, 1500, 3000]
    return trapmf(x, 0.0f, 0.0f, 1500.0f, 3000.0f);
}

float tt_sedang(float x) {
    // Trimf [2000, 4000, 6000]
    return trimf(x, 2000.0f, 4000.0f, 6000.0f);
}

float tt_lama(float x) {
    // Trapmf [5000, 8000, 10000, 10000]
    return trapmf(x, 5000.0f, 8000.0f, 10000.0f, 10000.0f);
}

// ============================================================
// Fungsi Keanggotaan Input OT
// ============================================================

float ot_singkat(float x) {
    // Trapmf [0, 0, 500, 1500]
    return trapmf(x, 0.0f, 0.0f, 500.0f, 1500.0f);
}

float ot_sedang(float x) {
    // Trimf [1000, 2000, 3000]
    return trimf(x, 1000.0f, 2000.0f, 3000.0f);
}

float ot_lama(float x) {
    // Trapmf [2500, 4000, 5000, 5000]
    return trapmf(x, 2500.0f, 4000.0f, 5000.0f, 5000.0f);
}

// ============================================================
// Fungsi Keanggotaan Output CL
// ============================================================

float cl_lancar(float x) {
    // Trapmf [0, 0, 10, 40]
    return trapmf(x, 0.0f, 0.0f, 10.0f, 40.0f);
}

float cl_padat(float x) {
    // Trimf [20, 50, 80]
    return trimf(x, 20.0f, 50.0f, 80.0f);
}

float cl_macet(float x) {
    // Trapmf [60, 80, 100, 100]
    return trapmf(x, 60.0f, 80.0f, 100.0f, 100.0f);
}

// ============================================================
// FIS Utama (Mamdani)
// ============================================================

float runFIS(float tt, float ot) {
    // Pastikan input dalam rentang normalisasi
    tt = constrain(tt, 0.0f, TT_MAX_MS);
    ot = constrain(ot, 0.0f, OT_MAX_MS);

    // ----------------------------------------------------------
    // Fuzzifikasi: hitung derajat keanggotaan untuk setiap input
    // ----------------------------------------------------------
    float tt_c = tt_cepat(tt);
    float tt_s = tt_sedang(tt);
    float tt_l = tt_lama(tt);

    float ot_sg = ot_singkat(ot);
    float ot_s  = ot_sedang(ot);
    float ot_l  = ot_lama(ot);

    // ----------------------------------------------------------
    // Inferensi: 9 aturan Mamdani (AND = MIN)
    // ----------------------------------------------------------
    // R1: TT Cepat  AND OT Singkat → Lancar
    float r1 = min(tt_c, ot_sg);
    // R2: TT Cepat  AND OT Sedang  → Lancar
    float r2 = min(tt_c, ot_s);
    // R3: TT Cepat  AND OT Lama    → Padat
    float r3 = min(tt_c, ot_l);
    // R4: TT Sedang AND OT Singkat → Lancar
    float r4 = min(tt_s, ot_sg);
    // R5: TT Sedang AND OT Sedang  → Padat
    float r5 = min(tt_s, ot_s);
    // R6: TT Sedang AND OT Lama    → Macet
    float r6 = min(tt_s, ot_l);
    // R7: TT Lama   AND OT Singkat → Padat (blind spot)
    float r7 = min(tt_l, ot_sg);
    // R8: TT Lama   AND OT Sedang  → Macet
    float r8 = min(tt_l, ot_s);
    // R9: TT Lama   AND OT Lama    → Macet
    float r9 = min(tt_l, ot_l);

    // ----------------------------------------------------------
    // Agregasi output: kluster per kategori
    // ----------------------------------------------------------
    // Lancar: MAX(R1, R2, R4)
    float strength_lancar = max(r1, max(r2, r4));
    // Padat: MAX(R3, R5, R7)
    float strength_padat = max(r3, max(r5, r7));
    // Macet: MAX(R6, R8, R9)
    float strength_macet = max(r6, max(r8, r9));

    // ----------------------------------------------------------
    // Defuzzifikasi: Center of Gravity (COG)
    // ----------------------------------------------------------
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (float z = COG_MIN; z <= COG_MAX; z += COG_STEP) {
        // Hitung keanggotaan output untuk setiap kategori
        float mu_lancar = cl_lancar(z);
        float mu_padat  = cl_padat(z);
        float mu_macet  = cl_macet(z);

        // Clipping: MIN dengan firing strength
        float clipped_lancar = min(mu_lancar, strength_lancar);
        float clipped_padat  = min(mu_padat, strength_padat);
        float clipped_macet  = min(mu_macet, strength_macet);

        // Agregasi: MAX dari semua kluster
        float mu_aggregated = max(clipped_lancar, max(clipped_padat, clipped_macet));

        numerator += z * mu_aggregated;
        denominator += mu_aggregated;
    }

    // Hindari division by zero
    if (denominator == 0.0f) {
        return 0.0f;
    }

    return numerator / denominator;
}

// ============================================================
// Klasifikasi Status berdasarkan CL
// ============================================================

StatusKemacetan classifyStatus(float cl_val) {
    if (cl_val <= CL_BATAS_LANCAR) {
        return LANCAR;
    } else if (cl_val <= CL_BATAS_PADAT) {
        return PADAT;
    } else {
        return MACET;
    }
}
