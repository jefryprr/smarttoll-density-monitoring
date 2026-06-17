#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

// ================================================================
//  MODUL DISPLAY — LCD 16x2 I2C
//
//  Baris 1: Status + tt_fis_input          -> "LANCAR  T:2.8s"
//  Baris 2: Jumlah kendaraan di tol + CL    -> "N:3     CL:12.5"
//
//  N = tt_count + tt2_count = total kendaraan yang sudah masuk S1
//      tapi belum keluar S3 (masih berada di dalam area tol).
// ================================================================
void updateLCD(int st, float tt_fis_ms, uint8_t vehicle_count, float cl_val);

#endif // DISPLAY_H
