#ifndef ACTUATORS_H
#define ACTUATORS_H

// ================================================================
//  MODUL AKTUATOR — LED, Buzzer, dan Servo Palang Masuk
// ================================================================

// Kontrol LED status + buzzer secara non-blocking (berbasis millis).
//   0 = LANCAR : Hijau ON
//   1 = PADAT  : Kuning ON, buzzer singkat tiap 800ms
//   2 = MACET  : Merah berkedip, buzzer alarm tiap 250ms
void controlIndicators(int st);

// State machine palang masuk. Default tertutup.
// Terbuka jika kendaraan stabil di S1 (debounced) DAN status != MACET.
// Tetap terbuka selama GATE_CLEARANCE_MS sebelum menutup kembali.
void controlServoMasuk(int st, bool det1_debounced);

#endif // ACTUATORS_H
