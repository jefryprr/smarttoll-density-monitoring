/**
 * globals.h — Deklarasi Variabel Global
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Semua variabel global yang digunakan lintas modul.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "config.h"

// ============================================================
// Jarak Sensor Ultrasonik (cm)
// ============================================================
extern float dist1;
extern float dist2;
extern float dist3;

// ============================================================
// Flag Deteksi Kendaraan (raw)
// ============================================================
extern bool detect1;
extern bool detect2;
extern bool detect3;

// ============================================================
// Flag Deteksi Ter-debounce (S1 saja)
// ============================================================
extern bool debounced1;

// ============================================================
// Flag Deteksi Sebelumnya (untuk rising edge)
// ============================================================
extern bool prev_detect1;
extern bool prev_detect2;
extern bool prev_detect3;

// ============================================================
// Occupancy Time per sensor (ms)
// ============================================================
extern unsigned long ot_s1;
extern unsigned long ot_s2;
extern unsigned long ot_s3;
extern unsigned long occupancy_time_ms;
extern unsigned long ot_s1_start;
extern unsigned long ot_s2_start;
extern unsigned long ot_s3_start;
extern bool ot_s1_active;
extern bool ot_s2_active;
extern bool ot_s3_active;

// ============================================================
// Buffer Antrian Travel Time (FIFO Circular)
// ============================================================
// TT1: S1 → S2
extern unsigned long tt_queue[TT_QUEUE_CAPACITY];
extern int tt_head;
extern int tt_tail;
extern int tt_count;

// TT2: S2 → S3
extern unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
extern int tt2_head;
extern int tt2_tail;
extern int tt2_count;

// ============================================================
// Hasil Travel Time (ms)
// ============================================================
extern unsigned long travel_time_ms;
extern unsigned long travel_time2_ms;
extern float tt_fis_input; // MAX(TT1, TT2) untuk FIS

// ============================================================
// Status dan Congestion Level
// ============================================================
extern StatusKemacetan status;
extern float cl; // Congestion Level output FIS

// ============================================================
// Timing dan Timestamp
// ============================================================
extern unsigned long lastDebounce;
extern unsigned long lastServoChange;
extern unsigned long lastBuzzerChange;
extern unsigned long lastBlinkChange;
extern unsigned long loopStartTime;
extern unsigned long lastLoopTime;

// ============================================================
// State Servo / Palang
// ============================================================
extern bool gateOpen;
extern unsigned long gateClearanceStart;
extern bool gateClearanceActive;

// ============================================================
// Counter Kendaraan
// ============================================================
extern unsigned long vehicle_count;

// ============================================================
// Counter Loop (untuk debug)
// ============================================================
extern unsigned long loop_count;

// ============================================================
// State LED non-blocking
// ============================================================
extern bool ledMerahState;
extern bool buzzerState;

// ============================================================
// Dominant TT (untuk display)
// ============================================================
extern float tt_dominant;

#endif // GLOBALS_H
