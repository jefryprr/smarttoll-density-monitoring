#include "travel_time.h"
#include "globals.h"
#include "config.h"

/**
 * Memperbarui Travel Time TT1 (S1→S2) dan TT2 (S2→S3).
 *
 * Satu Rising Edge di S2 memicu dua aksi sekaligus:
 *   - POP dari tt_queue  → TT1 final (kendaraan menyelesaikan S1→S2)
 *   - PUSH ke tt2_queue  → TT2 mulai (kendaraan masuk segmen S2→S3)
 * Sehingga tidak ada gap antara TT1 dan TT2 — seluruh jalur S1→S3
 * terpantau tanpa blind spot.
 */
void updateTravelTime() {
  unsigned long now = millis();

  // ── TT1 FASE 1: Rising Edge S1 → PUSH ke tt_queue ──
  if (detect1 == 1 && prev_detect1 == 0) {
    if (tt_count < TT_QUEUE_CAPACITY) {
      tt_queue[tt_tail] = now;
      tt_tail           = (tt_tail + 1) % TT_QUEUE_CAPACITY;
      tt_count++;
      Serial.print(F("  [TT1] Rising Edge S1 @ "));
      Serial.print(now);
      Serial.print(F(" ms -> PUSH. TT1 Antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT1] Antrean PENUH ("));
      Serial.print(TT_QUEUE_CAPACITY);
      Serial.println(F(" slot) - S1 Rising Edge diabaikan."));
    }
  }

  // ── TT1 FASE 2 + TT2 FASE 1: Rising Edge S2 ──
  //    AKSI 1 (TT1): POP dari tt_queue -> hitung TT1 final
  //    AKSI 2 (TT2): PUSH ke tt2_queue -> mulai stopwatch S2->S3
  if (detect2 == 1 && prev_detect2 == 0) {

    // AKSI 1 — Selesaikan TT1
    if (tt_count > 0) {
      unsigned long s1_ts = tt_queue[tt_head];
      tt_head             = (tt_head + 1) % TT_QUEUE_CAPACITY;
      tt_count--;
      float elapsed  = (float)(now - s1_ts);
      travel_time_ms = (elapsed > (float)TT_MAX_MS)
                       ? (float)TT_MAX_MS : elapsed;
      Serial.print(F("  [TT1] Rising Edge S2 -> POP. TT1 final = "));
      Serial.print(travel_time_ms, 0);
      Serial.print(F(" ms. Sisa TT1 antrean: "));
      Serial.print(tt_count);
      Serial.print(F("/"));
      Serial.println(TT_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT1] Rising Edge S2 tapi tt1 antrean kosong -> diabaikan."));
    }

    // AKSI 2 — Mulai TT2
    if (tt2_count < TT2_QUEUE_CAPACITY) {
      tt2_queue[tt2_tail] = now;
      tt2_tail            = (tt2_tail + 1) % TT2_QUEUE_CAPACITY;
      tt2_count++;
      Serial.print(F("  [TT2] Rising Edge S2 -> PUSH. TT2 Antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.print(F("  [TT2] Antrean PENUH ("));
      Serial.print(TT2_QUEUE_CAPACITY);
      Serial.println(F(" slot) - S2 PUSH diabaikan."));
    }
  }

  // ── TT2 FASE 2: Rising Edge S3 → POP, hitung TT2 final ──
  if (detect3 == 1 && prev_detect3 == 0) {
    if (tt2_count > 0) {
      unsigned long s2_ts = tt2_queue[tt2_head];
      tt2_head            = (tt2_head + 1) % TT2_QUEUE_CAPACITY;
      tt2_count--;
      float elapsed2  = (float)(now - s2_ts);
      travel_time2_ms = (elapsed2 > (float)TT_MAX_MS)
                        ? (float)TT_MAX_MS : elapsed2;
      Serial.print(F("  [TT2] Rising Edge S3 -> POP. TT2 final = "));
      Serial.print(travel_time2_ms, 0);
      Serial.print(F(" ms. Sisa TT2 antrean: "));
      Serial.print(tt2_count);
      Serial.print(F("/"));
      Serial.println(TT2_QUEUE_CAPACITY);
    } else {
      Serial.println(F("  [TT2] Rising Edge S3 tapi tt2 antrean kosong -> diabaikan."));
    }
  }

  // ── TT1 FASE 3: Live Tracking (PEEK kendaraan terdepan S1→S2) ──
  // Jika kendaraan mogok antara S1 dan S2, TT1 terus naik -> R7a aktif.
  if (tt_count > 0) {
    float elapsed  = (float)(now - tt_queue[tt_head]);
    travel_time_ms = (elapsed > (float)TT_MAX_MS)
                     ? (float)TT_MAX_MS : elapsed;
  }

  // ── TT2 FASE 3: Live Tracking (PEEK kendaraan terdepan S2→S3) ──
  // Jika kendaraan mogok antara S2 dan S3, TT2 terus naik -> R7b aktif.
  if (tt2_count > 0) {
    float elapsed2  = (float)(now - tt2_queue[tt2_head]);
    travel_time2_ms = (elapsed2 > (float)TT_MAX_MS)
                      ? (float)TT_MAX_MS : elapsed2;
  }

  // ── TT1 FASE 4: Idle Reset ──
  if (tt_count == 0 && detect1 == 0 && detect2 == 0 && detect3 == 0) {
    travel_time_ms = 0.0f;
  }

  // ── TT2 FASE 4: Idle Reset ──
  // Reset TT2 jika tidak ada kendaraan aktif di segmen S2->S3.
  if (tt2_count == 0 && detect2 == 0 && detect3 == 0) {
    travel_time2_ms = 0.0f;
  }
}
