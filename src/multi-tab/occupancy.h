#ifndef OCCUPANCY_H
#define OCCUPANCY_H

// ================================================================
//  MODUL OCCUPANCY TIME — Durasi sensor terbaca HIGH kontinu
//  OT untuk FIS = MAX(OT_S1, OT_S2, OT_S3) -> kondisi terparah.
//  OT panjang = kendaraan BERHENTI TEPAT di depan salah satu sensor.
// ================================================================
void updateOccupancyTime();

#endif // OCCUPANCY_H
