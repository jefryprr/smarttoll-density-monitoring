#include "fuzzy_logic.h"
#include "globals.h"
#include <math.h>

// ================================================================
//  MEMBERSHIP FUNCTIONS
// ================================================================

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

// ─── MF Input TT — domain [0, 10000] ms ───
// Berlaku untuk tt_fis_input = MAX(TT1, TT2)
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

// ================================================================
//  DEFUZZIFIKASI COG (step = 5, 21 iterasi)
// ================================================================

/**
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

    // Implikasi Mamdani (clipping — MIN)
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

// ================================================================
//  FIS MAMDANI MIKROSKOPIS — Inti Sistem
// ================================================================

float runFuzzyMamdani(float tt_val, float ot_val) {

  // ── Langkah 1: Fuzzifikasi ──
  float mu_TT_C  = mf_TT_Cepat(tt_val);
  float mu_TT_S  = mf_TT_Sedang(tt_val);
  float mu_TT_L  = mf_TT_Lama(tt_val);

  float mu_OT_Si = mf_OT_Singkat(ot_val);
  float mu_OT_Se = mf_OT_Sedang(ot_val);
  float mu_OT_L  = mf_OT_Lama(ot_val);

  // ── Langkah 2: Evaluasi Rule — Firing Strength (alpha = MIN) ──
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
    // Identifikasi blind spot mana yang dominan
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
