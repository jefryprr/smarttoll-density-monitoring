#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// ================================================================
//  MODUL SENSOR — Pembacaan HC-SR04 & Deteksi Kendaraan
// ================================================================

// Mengukur jarak satu sensor HC-SR04 (Time-of-Flight).
// Rumus: d [cm] = t_echo [µs] × 0.01715
// Return 30.0 cm jika timeout (tidak ada pantulan terdeteksi).
float readUltrasonic(int trigPin, int echoPin);

// Membaca ketiga sensor secara berurutan (time-multiplexed).
// SENSOR_DELAY mencegah cross-talk antar sensor. Hasil disimpan
// ke dist1 / dist2 / dist3.
void readAllSensors();

// Menentukan apakah kendaraan terdeteksi di depan sensor.
// Return 1 jika kendaraan terdeteksi, 0 jika tidak.
int detectVehicle(float distance_cm);

#endif // SENSORS_H
