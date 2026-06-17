#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include "config.h"

// ================================================================
//  STATE GLOBAL SISTEM
//  File ini hanya berisi deklarasi (extern) agar bisa diakses dari
//  semua modul .cpp. Definisi aktual ada di globals.cpp.
// ================================================================

// ---------------- Objek Perangkat ----------------
extern LiquidCrystal_I2C lcd;
extern Servo servoMasuk;

// ---------------- Data Sensor & Deteksi ----------------
extern float dist1, dist2, dist3;
extern int   detect1, detect2, detect3;
extern int   prev_detect1, prev_detect2, prev_detect3;

// ---------------- Debounce Sensor 1 ----------------
extern unsigned long detect1_stable_since;
extern bool          detect1_debounced;

// ---------------- FIFO Travel Time TT1 (S1→S2) ----------------
extern unsigned long tt_queue[TT_QUEUE_CAPACITY];
extern uint8_t        tt_head, tt_tail, tt_count;
extern float           travel_time_ms;

// ---------------- FIFO Travel Time TT2 (S2→S3) ----------------
extern unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
extern uint8_t         tt2_head, tt2_tail, tt2_count;
extern float            travel_time2_ms;

// ---------------- Input FIS Gabungan ----------------
// tt_fis_input = MAX(travel_time_ms, travel_time2_ms)
extern float tt_fis_input;

// ---------------- Occupancy Time ----------------
extern unsigned long occ1_start_ms, occ2_start_ms, occ3_start_ms;
extern bool           occ1_active,  occ2_active,  occ3_active;
extern float          occupancy_time_ms;

// ---------------- Output Fuzzy & Status Akhir ----------------
extern float CL;
extern int   status;

// ---------------- State Machine Servo Masuk ----------------
extern unsigned long lastGateOpenTime;
extern bool           isGateOpen;

// ---------------- Timing Non-blocking LED/Buzzer ----------------
extern unsigned long lastBlink;
extern bool           ledMerahState;
extern unsigned long lastBuzzer;

// ---------------- Monitoring ----------------
extern unsigned long loopCount;

#endif // GLOBALS_H
