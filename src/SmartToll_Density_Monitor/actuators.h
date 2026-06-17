/**
 * actuators.h — Deklarasi Fungsi Aktuator
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Mengelola LED, buzzer, dan servo palang secara non-blocking.
 */

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"
#include "globals.h"

// Interval kedip LED merah (ms)
#define LED_BLINK_INTERVAL 300

// Durasi beep buzzer (ms)
#define BUZZER_BEEP_PADAT  800
#define BUZZER_BEEP_MACET  250

/**
 * Menginisialisasi semua aktuator:
 * - Pin LED sebagai OUTPUT
 * - Pin buzzer sebagai OUTPUT
 * - Servo.attach(SERVO_PIN)
 */
void initActuators();

/**
 * Memperbarui LED sesuai status:
 * - LANCAR: Hijau menyala terus, lainnya mati
 * - PADAT:  Kuning menyala terus, lainnya mati
 * - MACET:  Merah berkedip (non-blocking 300ms)
 */
void updateLEDs(StatusKemacetan st);

/**
 * Memperbarui buzzer sesuai status (non-blocking):
 * - LANCAR: Mati
 * - PADAT:  Beep 800ms ON, lalu OFF
 * - MACET:  Beep 250ms ON, lalu OFF
 */
void updateBuzzer(StatusKemacetan st);

/**
 * Mengontrol servo palang (state machine):
 * - Buka jika kendaraan terdeteksi di S1 DAN status ≠ MACET
 * - Gate clearance 200ms sebelum berpindah state
 * - Tutup jika tidak ada kendaraan atau status MACET
 */
void updateServo();

#endif // ACTUATORS_H
