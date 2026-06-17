/**
 * occupancy.cpp — Implementasi Fungsi Occupancy Time
 * SmartToll Density Monitoring System v3.2.1
 */

#include "occupancy.h"

// ============================================================
// Memperbarui occupancy time untuk setiap sensor
// ============================================================
void updateOccupancy() {
    unsigned long now = millis();

    // --- Sensor S1 ---
    if (detect1) {
        if (!ot_s1_active) {
            // Baru mulai mendeteksi → mulai timer
            ot_s1_start = now;
            ot_s1_active = true;
        }
        ot_s1 = now - ot_s1_start;
    } else {
        // Sensor tidak mendeteksi → reset
        ot_s1_active = false;
        ot_s1 = 0;
    }

    // --- Sensor S2 ---
    if (detect2) {
        if (!ot_s2_active) {
            ot_s2_start = now;
            ot_s2_active = true;
        }
        ot_s2 = now - ot_s2_start;
    } else {
        ot_s2_active = false;
        ot_s2 = 0;
    }

    // --- Sensor S3 ---
    if (detect3) {
        if (!ot_s3_active) {
            ot_s3_start = now;
            ot_s3_active = true;
        }
        ot_s3 = now - ot_s3_start;
    } else {
        ot_s3_active = false;
        ot_s3 = 0;
    }

    // occupancy_time_ms = MAX(ot_s1, ot_s2, ot_s3)
    occupancy_time_ms = ot_s1;
    if (ot_s2 > occupancy_time_ms) occupancy_time_ms = ot_s2;
    if (ot_s3 > occupancy_time_ms) occupancy_time_ms = ot_s3;
}
