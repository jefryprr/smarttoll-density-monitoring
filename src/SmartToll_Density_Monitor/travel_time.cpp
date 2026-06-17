/**
 * travel_time.cpp — Implementasi Fungsi Travel Time
 * SmartToll Density Monitoring System v3.2.1
 */

#include "travel_time.h"
#include "sensors.h"

// Timeout untuk ghost vehicle eviction (30 detik)
#define GHOST_TIMEOUT_MS 30000UL

// ============================================================
// Operasi Buffer TT1 (S1 → S2)
// ============================================================

bool pushTT1(unsigned long timestamp) {
    if (tt_count >= TT_QUEUE_CAPACITY) {
        return false; // Buffer penuh
    }
    tt_queue[tt_tail] = timestamp;
    tt_tail = (tt_tail + 1) % TT_QUEUE_CAPACITY;
    tt_count++;
    return true;
}

bool popTT1(unsigned long &elapsed) {
    if (tt_count <= 0) {
        return false; // Buffer kosong
    }
    unsigned long stored_ts = tt_queue[tt_head];
    elapsed = millis() - stored_ts;
    tt_head = (tt_head + 1) % TT_QUEUE_CAPACITY;
    tt_count--;
    return true;
}

unsigned long peekTT1() {
    if (tt_count <= 0) {
        return 0;
    }
    return millis() - tt_queue[tt_head];
}

// ============================================================
// Operasi Buffer TT2 (S2 → S3)
// ============================================================

bool pushTT2(unsigned long timestamp) {
    if (tt2_count >= TT2_QUEUE_CAPACITY) {
        return false; // Buffer penuh
    }
    tt2_queue[tt2_tail] = timestamp;
    tt2_tail = (tt2_tail + 1) % TT2_QUEUE_CAPACITY;
    tt2_count++;
    return true;
}

bool popTT2(unsigned long &elapsed) {
    if (tt2_count <= 0) {
        return false; // Buffer kosong
    }
    unsigned long stored_ts = tt2_queue[tt2_head];
    elapsed = millis() - stored_ts;
    tt2_head = (tt2_head + 1) % TT2_QUEUE_CAPACITY;
    tt2_count--;
    return true;
}

unsigned long peekTT2() {
    if (tt2_count <= 0) {
        return 0;
    }
    return millis() - tt2_queue[tt2_head];
}

// ============================================================
// Logika Utama Travel Time
// ============================================================

void updateTravelTime() {
    // Rising edge S1 → push timestamp ke buffer TT1
    if (isRisingEdge(detect1, prev_detect1)) {
        pushTT1(millis());
    }

    // Rising edge S2 → pop TT1 + push timestamp ke buffer TT2
    if (isRisingEdge(detect2, prev_detect2)) {
        unsigned long elapsed1 = 0;
        if (popTT1(elapsed1)) {
            travel_time_ms = elapsed1;
        }
        pushTT2(millis());
    }

    // Rising edge S3 → pop TT2
    if (isRisingEdge(detect3, prev_detect3)) {
        unsigned long elapsed2 = 0;
        if (popTT2(elapsed2)) {
            travel_time2_ms = elapsed2;
        }
        vehicle_count++;
    }

    // Idle reset: jika buffer kosong dan semua sensor tidak deteksi
    if (tt_count == 0 && tt2_count == 0 && !detect1 && !detect2 && !detect3) {
        travel_time_ms = 0;
        travel_time2_ms = 0;
    }

    // Ghost vehicle eviction
    evictGhosts(GHOST_TIMEOUT_MS);
}

// ============================================================
// Ghost Vehicle Eviction
// ============================================================

void evictGhosts(unsigned long threshold_ms) {
    unsigned long now = millis();

    // Evict dari TT1
    while (tt_count > 0) {
        unsigned long age = now - tt_queue[tt_head];
        if (age > threshold_ms) {
            // Buang entry yang terlalu lama
            tt_head = (tt_head + 1) % TT_QUEUE_CAPACITY;
            tt_count--;
            // Set travel_time ke 0 karena ini ghost
            travel_time_ms = 0;
        } else {
            break;
        }
    }

    // Evict dari TT2
    while (tt2_count > 0) {
        unsigned long age = now - tt2_queue[tt2_head];
        if (age > threshold_ms) {
            tt2_head = (tt2_head + 1) % TT2_QUEUE_CAPACITY;
            tt2_count--;
            travel_time2_ms = 0;
        } else {
            break;
        }
    }
}
