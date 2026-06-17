/**
 * sensors.cpp — Implementasi Fungsi Sensor Ultrasonik
 * SmartToll Density Monitoring System v3.2.1
 */

#include "sensors.h"

// ============================================================
// Membaca jarak dari satu sensor ultrasonik
// ============================================================
float readUltrasonic(int trigPin, int echoPin) {
    // Kirim pulsa trigger 10us
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Baca durasi echo (timeout 30ms ≈ ~500cm)
    long duration = pulseIn(echoPin, HIGH, 30000UL);

    // Hitung jarak: durasi × kecepatan_suara / 2
    // Kecepatan suara ≈ 0.034 cm/μs
    if (duration == 0) {
        return -1.0f; // Timeout — tidak ada objek terdeteksi
    }

    float distance = (float)duration * 0.034f / 2.0f;
    return distance;
}

// ============================================================
// Membaca ketiga sensor dan memperbarui flag deteksi
// ============================================================
void readAllSensors() {
    // Baca sensor S1
    dist1 = readUltrasonic(S1_TRIG, S1_ECHO);
    delay(SENSOR_DELAY);

    // Baca sensor S2
    dist2 = readUltrasonic(S2_TRIG, S2_ECHO);
    delay(SENSOR_DELAY);

    // Baca sensor S3
    dist3 = readUltrasonic(S3_TRIG, S3_ECHO);
    delay(SENSOR_DELAY);

    // Deteksi kendaraan: jarak valid dan di bawah ambang
    detect1 = (dist1 > 0.0f && dist1 < BATAS_LEBAR_JALAN);
    detect2 = (dist2 > 0.0f && dist2 < BATAS_LEBAR_JALAN);
    detect3 = (dist3 > 0.0f && dist3 < BATAS_LEBAR_JALAN);
}

// ============================================================
// Debounce sensor S1
// ============================================================
void updateDebounce() {
    static bool lastRaw1 = false;
    static unsigned long debounceStart = 0;
    static bool debouncedState = false;

    bool currentRaw = detect1;

    // Jika state berubah, mulai timer debounce
    if (currentRaw != lastRaw1) {
        debounceStart = millis();
        lastRaw1 = currentRaw;
    }

    // Jika sudah melewati waktu debounce, terima state baru
    if ((millis() - debounceStart) >= DEBOUNCE_MS) {
        debouncedState = currentRaw;
    }

    debounced1 = debouncedState;
}

// ============================================================
// Rising edge detection helper
// ============================================================
bool isRisingEdge(bool current, bool prev) {
    return (current && !prev);
}
