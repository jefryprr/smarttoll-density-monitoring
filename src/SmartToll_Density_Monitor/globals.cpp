/**
 * globals.cpp — Definisi Variabel Global
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Implementasi semua variabel global yang dideklarasikan di globals.h.
 */

#include "globals.h"

// ============================================================
// Jarak Sensor Ultrasonik (cm)
// ============================================================
float dist1 = 0.0f;
float dist2 = 0.0f;
float dist3 = 0.0f;

// ============================================================
// Flag Deteksi Kendaraan (raw)
// ============================================================
bool detect1 = false;
bool detect2 = false;
bool detect3 = false;

// ============================================================
// Flag Deteksi Ter-debounce (S1 saja)
// ============================================================
bool debounced1 = false;

// ============================================================
// Flag Deteksi Sebelumnya (untuk rising edge)
// ============================================================
bool prev_detect1 = false;
bool prev_detect2 = false;
bool prev_detect3 = false;

// ============================================================
// Occupancy Time per sensor (ms)
// ============================================================
unsigned long ot_s1 = 0;
unsigned long ot_s2 = 0;
unsigned long ot_s3 = 0;
unsigned long occupancy_time_ms = 0;
unsigned long ot_s1_start = 0;
unsigned long ot_s2_start = 0;
unsigned long ot_s3_start = 0;
bool ot_s1_active = false;
bool ot_s2_active = false;
bool ot_s3_active = false;

// ============================================================
// Buffer Antrian Travel Time (FIFO Circular)
// ============================================================
// TT1: S1 → S2
unsigned long tt_queue[TT_QUEUE_CAPACITY];
int tt_head = 0;
int tt_tail = 0;
int tt_count = 0;

// TT2: S2 → S3
unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
int tt2_head = 0;
int tt2_tail = 0;
int tt2_count = 0;

// ============================================================
// Hasil Travel Time (ms)
// ============================================================
unsigned long travel_time_ms = 0;
unsigned long travel_time2_ms = 0;
float tt_fis_input = 0.0f;

// ============================================================
// Status dan Congestion Level
// ============================================================
StatusKemacetan status = LANCAR;
float cl = 0.0f;

// ============================================================
// Timing dan Timestamp
// ============================================================
unsigned long lastDebounce = 0;
unsigned long lastServoChange = 0;
unsigned long lastBuzzerChange = 0;
unsigned long lastBlinkChange = 0;
unsigned long loopStartTime = 0;
unsigned long lastLoopTime = 0;

// ============================================================
// State Servo / Palang
// ============================================================
bool gateOpen = false;
unsigned long gateClearanceStart = 0;
bool gateClearanceActive = false;

// ============================================================
// Counter Kendaraan
// ============================================================
unsigned long vehicle_count = 0;

// ============================================================
// Counter Loop (untuk debug)
// ============================================================
unsigned long loop_count = 0;

// ============================================================
// State LED non-blocking
// ============================================================
bool ledMerahState = false;
bool buzzerState = false;

// ============================================================
// Dominant TT (untuk display)
// ============================================================
float tt_dominant = 0.0f;
