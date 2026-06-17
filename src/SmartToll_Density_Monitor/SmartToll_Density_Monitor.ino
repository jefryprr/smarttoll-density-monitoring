/**
 * SmartToll_Density_Monitor.ino — Program Utama
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Sistem monitoring kepadatan lalu lintas berbasis ESP32
 * menggunakan Fuzzy Inference System (Mamdani) untuk
 * mengklasifikasikan tingkat kemacetan dari Travel Time (TT)
 * dan Occupancy Time (OT).
 * 
 * Platform  : ESP32
 * Framework : Arduino
 * 
 * Tahapan loop (T1–T13):
 *   T1: Baca sensor ultrasonik
 *   T2: Debounce sensor S1
 *   T3: Update Travel Time (TT1, TT2)
 *   T4: Update Occupancy Time (OT)
 *   T5: Hitung tt_fis_input = MAX(TT1, TT2)
 *   T6: Jalankan FIS (Fuzzy Inference System)
 *   T7: Klasifikasi status (LANCAR/PADAT/MACET)
 *   T8: Kontrol indikator (LED + Buzzer)
 *   T9: Kontrol servo palang
 *   T10: Update LCD
 *   T11: Cetak serial debug
 *   T12: Update prev_detect flags
 *   T13: Timing control (LOOP_DELAY)
 */

// ============================================================
// Include Libraries
// ============================================================
#include <Wire.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>

// Include modul sistem
#include "config.h"
#include "globals.h"
#include "sensors.h"
#include "travel_time.h"
#include "occupancy.h"
#include "fuzzy_logic.h"
#include "actuators.h"
#include "display.h"
#include "debug_serial.h"

// ============================================================
// Setup
// ============================================================
void setup() {
    // --- Inisialisasi Serial ---
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println(F(""));
    Serial.println(F("========================================"));
    Serial.println(F(" SmartToll Density Monitor v3.2.1"));
    Serial.println(F(" ESP32 + FIS Mamdani"));
    Serial.println(F("========================================"));
    Serial.println(F(""));

    // --- Inisialisasi I2C ---
    Wire.begin(21, 22); // SDA=21, SCL=22

    // --- Inisialisasi Pin Sensor Ultrasonik ---
    pinMode(S1_TRIG, OUTPUT);
    pinMode(S1_ECHO, INPUT);
    pinMode(S2_TRIG, OUTPUT);
    pinMode(S2_ECHO, INPUT);
    pinMode(S3_TRIG, OUTPUT);
    pinMode(S3_ECHO, INPUT);

    // --- Inisialisasi Aktuator ---
    initActuators();

    // --- Inisialisasi LCD ---
    initLCD();

    // --- Inisialisasi Buffer ---
    tt_head = 0;
    tt_tail = 0;
    tt_count = 0;
    tt2_head = 0;
    tt2_tail = 0;
    tt2_count = 0;

    // --- Inisialisasi Variabel ---
    travel_time_ms = 0;
    travel_time2_ms = 0;
    tt_fis_input = 0.0f;
    cl = 0.0f;
    status = LANCAR;
    vehicle_count = 0;
    loop_count = 0;

    // Inisialisasi timing
    lastDebounce = millis();
    lastServoChange = millis();
    lastBuzzerChange = millis();
    lastBlinkChange = millis();
    lastLoopTime = millis();

    Serial.println(F("[SETUP] Semua modul terinisialisasi."));
    Serial.println(F("[SETUP] Memulai loop utama..."));
    Serial.println(F(""));
}

// ============================================================
// Loop Utama
// ============================================================
void loop() {
    loopStartTime = millis();
    loop_count++;

    // ==========================================================
    // T1: Baca Sensor Ultrasonik
    // ==========================================================
    readAllSensors();

    // ==========================================================
    // T2: Debounce Sensor S1
    // ==========================================================
    updateDebounce();

    // ==========================================================
    // T3: Update Travel Time (TT1, TT2)
    // ==========================================================
    updateTravelTime();

    // ==========================================================
    // T4: Update Occupancy Time (OT)
    // ==========================================================
    updateOccupancy();

    // ==========================================================
    // T5: Hitung tt_fis_input = MAX(TT1, TT2)
    // ==========================================================
    tt_fis_input = (float)((travel_time_ms > travel_time2_ms)
                           ? travel_time_ms
                           : travel_time2_ms);
    tt_dominant = tt_fis_input;

    // ==========================================================
    // T6: Jalankan FIS
    // ==========================================================
    cl = runFIS(tt_fis_input, (float)occupancy_time_ms);

    // ==========================================================
    // T7: Klasifikasi Status
    // ==========================================================
    status = classifyStatus(cl);

    // ==========================================================
    // T8: Kontrol Indikator (LED + Buzzer)
    // ==========================================================
    updateLEDs(status);
    updateBuzzer(status);

    // ==========================================================
    // T9: Kontrol Servo Palang
    // ==========================================================
    updateServo();

    // ==========================================================
    // T10: Update LCD
    // ==========================================================
    updateLCD(status, tt_dominant, vehicle_count, cl);

    // ==========================================================
    // T11: Cetak Serial Debug
    // ==========================================================
    serialPrintAll(loop_count);

    // ==========================================================
    // T12: Update prev_detect flags (untuk rising edge)
    // ==========================================================
    prev_detect1 = detect1;
    prev_detect2 = detect2;
    prev_detect3 = detect3;

    // ==========================================================
    // T13: Timing Control
    // ==========================================================
    unsigned long elapsed = millis() - loopStartTime;
    if (elapsed < LOOP_DELAY) {
        delay(LOOP_DELAY - elapsed);
    }
    lastLoopTime = millis();
}
