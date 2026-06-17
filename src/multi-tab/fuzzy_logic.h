#ifndef FUZZY_LOGIC_H
#define FUZZY_LOGIC_H

// ================================================================
//  MODUL FUZZY LOGIC — FIS Mamdani 9-Rule + Defuzzifikasi COG
//
//  INPUT 1: TT = tt_fis_input = MAX(TT1[S1->S2], TT2[S2->S3])
//  INPUT 2: OT = occupancy_time_ms
//  OUTPUT : CL = Congestion Level [0..100]
//
//  Rule Base (AND = MIN, agregasi output = MAX):
//    R1 Cepat  ∧ Singkat -> Lancar      R6 Sedang ∧ Lama    -> Macet
//    R2 Cepat  ∧ Sedang  -> Lancar      R7 Lama   ∧ Singkat -> Macet *
//    R3 Cepat  ∧ Lama    -> Padat       R8 Lama   ∧ Sedang  -> Macet
//    R4 Sedang ∧ Singkat -> Lancar      R9 Lama   ∧ Lama    -> Macet
//    R5 Sedang ∧ Sedang  -> Padat
//
//  * R7 adalah rule deteksi blind spot: TT lama tapi OT singkat
//    berarti kendaraan mogok di TENGAH segmen (tidak menghalangi
//    sensor manapun). Karena TT = MAX(TT1, TT2), R7 otomatis
//    mencakup S1->S2 (R7a) maupun S2->S3 (R7b) tanpa perlu rule
//    tambahan — lihat komentar di runFuzzyMamdani().
// ================================================================

// --- Membership Functions Generik ---
float triangularMF(float x, float a, float b, float c);
float trapezoidalMF(float x, float a, float b, float c, float d);

// --- MF Input TT (domain 0..10000 ms) ---
float mf_TT_Cepat(float x);
float mf_TT_Sedang(float x);
float mf_TT_Lama(float x);

// --- MF Input OT (domain 0..5000 ms) ---
float mf_OT_Singkat(float x);
float mf_OT_Sedang(float x);
float mf_OT_Lama(float x);

// --- MF Output CL (domain 0..100) ---
float mf_CL_Lancar(float z);
float mf_CL_Padat(float z);
float mf_CL_Macet(float z);

// --- Defuzzifikasi Centre of Gravity (step = 5, 21 iterasi) ---
float defuzzifyCOG(float a_lancar, float a_padat, float a_macet);

// --- Pipeline FIS Lengkap: [TT, OT] -> CL* ---
float runFuzzyMamdani(float tt_val, float ot_val);

#endif // FUZZY_LOGIC_H
