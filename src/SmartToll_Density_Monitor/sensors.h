/**
 * sensors.h — Deklarasi Fungsi Sensor Ultrasonik
 * SmartToll Density Monitoring System v3.2.1
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"
#include "globals.h"

/**
 * Membaca jarak dari sensor ultrasonik HC-SR04.
 * @param trigPin  Pin trigger
 * @param echoPin  Pin echo
 * @return Jarak dalam cm, atau -1.0 jika timeout
 */
float readUltrasonic(int trigPin, int echoPin);

/**
 * Membaca ketiga sensor ultrasonik (S1, S2, S3).
 * Menyimpan hasil ke dist1, dist2, dist3.
 * Mendeteksi kendaraan jika jarak < BATAS_LEBAR_JALAN.
 */
void readAllSensors();

/**
 * Melakukan debounce pada sensor S1.
 * Menggunakan DEBOUNCE_MS sebagai waktu stabilisasi.
 */
void updateDebounce();

/**
 * Mengecek rising edge (transisi LOW→HIGH) pada flag deteksi.
 * @param current  Flag deteksi saat ini
 * @param prev     Flag deteksi sebelumnya
 * @return true jika terjadi rising edge
 */
bool isRisingEdge(bool current, bool prev);

#endif // SENSORS_H
