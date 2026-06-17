/**
 * debug_serial.h — Deklarasi Fungsi Debug Serial
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Menampilkan log debug lengkap ke Serial Monitor
 * sesuai format SERIAL_MONITOR.md.
 */

#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "config.h"
#include "globals.h"

/**
 * Mencetak semua data debug ke Serial Monitor.
 * Format lengkap mencakup:
 * - Header loop
 * - Pembacaan sensor
 * - Variabel mikroskopik
 * - Fuzzifikasi FIS
 * - Firing strength aturan
 * - Hasil COG
 * - Output sistem
 * 
 * @param loop_num Nomor loop saat ini
 */
void serialPrintAll(unsigned long loop_num);

#endif // DEBUG_SERIAL_H
