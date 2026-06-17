/**
 * occupancy.h — Deklarasi Fungsi Occupancy Time
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Menghitung berapa lama setiap sensor terus-menerus diblokir
 * oleh kendaraan (occupancy time).
 */

#ifndef OCCUPANCY_H
#define OCCUPANCY_H

#include <Arduino.h>
#include "config.h"
#include "globals.h"

/**
 * Memperbarui occupancy time untuk setiap sensor.
 * - Jika sensor mendeteksi kendaraan, mulai/mulai timer.
 * - Jika sensor tidak mendeteksi, reset timer ke 0.
 * - occupancy_time_ms = MAX(ot_s1, ot_s2, ot_s3)
 */
void updateOccupancy();

#endif // OCCUPANCY_H
