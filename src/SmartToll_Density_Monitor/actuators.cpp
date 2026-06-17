/**
 * actuators.cpp — Implementasi Fungsi Aktuator
 * SmartToll Density Monitoring System v3.2.1
 */

#include "actuators.h"

// Objek servo
static Servo gateServo;

// ============================================================
// Inisialisasi Aktuator
// ============================================================
void initActuators() {
    // LED
    pinMode(LED_HIJAU, OUTPUT);
    pinMode(LED_KUNING, OUTPUT);
    pinMode(LED_MERAH, OUTPUT);

    // Matikan semua LED awal
    digitalWrite(LED_HIJAU, LOW);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH, LOW);

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Servo
    gateServo.attach(SERVO_PIN);
    gateServo.write(SERVO_TUTUP);
    gateOpen = false;
}

// ============================================================
// Update LED berdasarkan Status
// ============================================================
void updateLEDs(StatusKemacetan st) {
    unsigned long now = millis();

    switch (st) {
        case LANCAR:
            // Hijau steady, lainnya mati
            digitalWrite(LED_HIJAU, HIGH);
            digitalWrite(LED_KUNING, LOW);
            digitalWrite(LED_MERAH, LOW);
            break;

        case PADAT:
            // Kuning steady, lainnya mati
            digitalWrite(LED_HIJAU, LOW);
            digitalWrite(LED_KUNING, HIGH);
            digitalWrite(LED_MERAH, LOW);
            break;

        case MACET:
            // Merah berkedip non-blocking
            digitalWrite(LED_HIJAU, LOW);
            digitalWrite(LED_KUNING, LOW);
            if (now - lastBlinkChange >= LED_BLINK_INTERVAL) {
                ledMerahState = !ledMerahState;
                digitalWrite(LED_MERAH, ledMerahState ? HIGH : LOW);
                lastBlinkChange = now;
            }
            break;
    }
}

// ============================================================
// Update Buzzer berdasarkan Status (non-blocking)
// ============================================================
void updateBuzzer(StatusKemacetan st) {
    unsigned long now = millis();

    switch (st) {
        case LANCAR:
            // Buzzer mati
            digitalWrite(BUZZER_PIN, LOW);
            buzzerState = false;
            break;

        case PADAT:
            // Beep 800ms, lalu mati
            if (!buzzerState) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerState = true;
                lastBuzzerChange = now;
            } else if (now - lastBuzzerChange >= BUZZER_BEEP_PADAT) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerState = false;
            }
            break;

        case MACET:
            // Beep 250ms, lalu mati
            if (!buzzerState) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerState = true;
                lastBuzzerChange = now;
            } else if (now - lastBuzzerChange >= BUZZER_BEEP_MACET) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerState = false;
            }
            break;
    }
}

// ============================================================
// Update Servo Palang (State Machine)
// ============================================================
void updateServo() {
    unsigned long now = millis();

    // Kondisi buka: kendaraan di S1 DAN status bukan MACET
    bool shouldOpen = debounced1 && (status != MACET);

    if (shouldOpen && !gateOpen) {
        // Perlu buka palang
        if (!gateClearanceActive) {
            // Mulai gate clearance timer
            gateClearanceActive = true;
            gateClearanceStart = now;
        } else if (now - gateClearanceStart >= GATE_CLEARANCE_MS) {
            // Clearance selesai → buka palang
            gateServo.write(SERVO_BUKA);
            gateOpen = true;
            gateClearanceActive = false;
        }
    } else if (!shouldOpen && gateOpen) {
        // Perlu tutup palang
        if (!gateClearanceActive) {
            gateClearanceActive = true;
            gateClearanceStart = now;
        } else if (now - gateClearanceStart >= GATE_CLEARANCE_MS) {
            // Clearance selesai → tutup palang
            gateServo.write(SERVO_TUTUP);
            gateOpen = false;
            gateClearanceActive = false;
        }
    } else {
        // Tidak perlu perubahan → reset clearance
        gateClearanceActive = false;
    }
}
