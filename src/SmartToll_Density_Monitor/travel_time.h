#ifndef TRAVEL_TIME_H
#define TRAVEL_TIME_H

// ================================================================
//  MODUL TRAVEL TIME — Dual FIFO TT1 (S1→S2) & TT2 (S2→S3)
//
//  TT1 : PUSH @ S1 rising edge   POP @ S2 rising edge
//  TT2 : PUSH @ S2 rising edge   POP @ S3 rising edge
//
//  Live tracking (PEEK) menjaga TT tetap naik selama kendaraan
//  terdepan belum melewati sensor berikutnya — inilah mekanisme
//  yang mendeteksi kendaraan mogok di blind spot S1–S2 (Rule R7a)
//  maupun S2–S3 (Rule R7b). Lihat fuzzy_logic.h untuk rule base.
// ================================================================
void updateTravelTime();

#endif // TRAVEL_TIME_H
