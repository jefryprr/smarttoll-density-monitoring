/**
 * ================================================================
 *  SmartToll Density Monitoring System — v3.2.1 (ESP32 Port)
 *  Tugas Besar Sistem Kendali dan Sensor Aktuator
 * ================================================================
 *  Dokumentasi lengkap (arsitektur, wiring, changelog v3.1 -> 3.2
 *  -> 3.2.1) ada di README.md pada root repo ini.
 *
 *  Paradigma: Discrete Event System (DES) mikroskopis.
 *    INPUT 1 (TT) : tt_fis_input = MAX(TT1[S1->S2], TT2[S2->S3])
 *    INPUT 2 (OT) : occupancy_time_ms = MAX(OT_S1, OT_S2, OT_S3)
 *    OUTPUT       : CL (Congestion Level) via FIS Mamdani 9-rule
 *
 *  Struktur file (Arduino multi-tab):
 *    config.h        konstanta pin & parameter sistem
 *    globals.h/.cpp  state global bersama seluruh modul
 *    sensors.*       pembacaan HC-SR04 & deteksi kendaraan
 *    travel_time.*   dual FIFO TT1 (S1->S2) & TT2 (S2->S3)
 *    occupancy.*     durasi okupansi per sensor
 *    fuzzy_logic.*   FIS Mamdani 9-rule + defuzzifikasi COG
 *    actuators.*     LED, buzzer, servo palang masuk
 *    display.*       LCD 16x2 I2C
 *    debug_serial.*  log lengkap ke Serial Monitor
 * ================================================================
 */

#include <Wire.h>
#include "config.h"
#include "globals.h"
#include "sensors.h"
#include "travel_time.h"
#include "occupancy.h"
#include "fuzzy_logic.h"
#include "actuators.h"
#include "display.h"
#include "debug_serial.h"

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("==============================================="));
  Serial.println(F("  SmartToll Density Monitoring -- v3.2.1"));
  Serial.println(F("  ESP32 DevKit | Dual FIFO TT (S1->S3)"));
  Serial.println(F("==============================================="));

  Wire.begin(21, 22);   // SDA, SCL -- default ESP32

  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT); pinMode(ECHO3, INPUT);

  pinMode(LED_HIJAU,  OUTPUT);
  pinMode(LED_KUNING, OUTPUT);
  pinMode(LED_MERAH,  OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_HIJAU,  LOW);
  digitalWrite(LED_KUNING, LOW);
  digitalWrite(LED_MERAH,  LOW);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("SmartToll v3.2.1"));
  lcd.setCursor(0, 1); lcd.print(F("Dual TT | N veh. "));

  servoMasuk.attach(SERVO_MASUK_PIN);
  servoMasuk.write(SERVO_TUTUP);   // default: tertutup

  // ── Animasi startup LED ──
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_HIJAU,  HIGH); delay(150);
    digitalWrite(LED_HIJAU,  LOW);
    digitalWrite(LED_KUNING, HIGH); delay(150);
    digitalWrite(LED_KUNING, LOW);
    digitalWrite(LED_MERAH,  HIGH); delay(150);
    digitalWrite(LED_MERAH,  LOW);
  }

  // ── Reset ulang seluruh state (defensive, walau sudah punya
  //    nilai default dari globals.cpp) ──
  memset(tt_queue, 0, sizeof(tt_queue));
  tt_head = tt_tail = tt_count = 0;
  travel_time_ms = 0.0f;

  memset(tt2_queue, 0, sizeof(tt2_queue));
  tt2_head = tt2_tail = tt2_count = 0;
  travel_time2_ms = 0.0f;
  tt_fis_input    = 0.0f;

  prev_detect1 = prev_detect2 = prev_detect3 = 0;

  occ1_active = occ2_active = occ3_active = false;
  occupancy_time_ms = 0.0f;

  detect1_stable_since = 0;
  detect1_debounced    = false;

  isGateOpen       = false;
  lastGateOpenTime = 0;

  CL     = 0.0f;
  status = 0;

  delay(800);
  lcd.clear();

  Serial.println(F("  Sistem siap. v3.2.1 ESP32 aktif!"));
  Serial.println(F("    S1 TRIG=13 ECHO=12 | S2 TRIG=14 ECHO=27 | S3 TRIG=26 ECHO=25"));
  Serial.println(F("    Servo=GPIO5 | LED Hijau=18 Kuning=19 Merah=23 | Buzzer=4"));
  Serial.println(F("    LCD I2C SDA=21 SCL=22"));
  Serial.println();
}

/**
 * Urutan eksekusi tiap siklus loop (~LOOP_DELAY ms):
 *   T1  Baca sensor                  T8  Kontrol LED + Buzzer
 *   T2  Debounce S1                  T9  Kontrol Servo Masuk
 *   T3  Update TT1 + TT2             T10 Update LCD
 *   T4  Update Occupancy Time        T11 Output Serial Monitor
 *   T5  tt_fis_input = MAX(TT1,TT2)  T12 Perbarui prev_detect
 *   T6  FIS Mamdani -> CL            T13 Timing control
 *   T7  Tentukan status
 */
void loop() {
  loopCount++;
  unsigned long loop_start = millis();

  // T1 — Baca semua sensor ultrasonik
  readAllSensors();
  detect1 = detectVehicle(dist1);
  detect2 = detectVehicle(dist2);
  detect3 = detectVehicle(dist3);

  // T2 — Debounce Sensor 1
  unsigned long now_db = millis();
  if (detect1 == 1) {
    if (detect1_stable_since == 0) detect1_stable_since = now_db;
    if ((now_db - detect1_stable_since) >= DEBOUNCE_MS) {
      detect1_debounced = true;
    }
  } else {
    detect1_stable_since = 0;
    detect1_debounced    = false;
  }

  // T3 — Update Travel Time (TT1 + TT2)
  updateTravelTime();

  // T4 — Update Occupancy Time (OT)
  updateOccupancyTime();

  // T5 — tt_fis_input = MAX(TT1, TT2): segmen terburuk masuk FIS
  tt_fis_input = (travel_time_ms > travel_time2_ms)
                 ? travel_time_ms : travel_time2_ms;

  // T6 — Jalankan FIS Mamdani -> CL
  CL = runFuzzyMamdani(tt_fis_input, occupancy_time_ms);

  // T7 — Tentukan status akhir
  if      (CL < CL_BATAS_LANCAR) status = 0;   // LANCAR
  else if (CL < CL_BATAS_PADAT)  status = 1;   // PADAT
  else                            status = 2;   // MACET

  // T8 — Kontrol indikator (LED + buzzer)
  controlIndicators(status);

  // T9 — Kontrol servo masuk (state machine)
  controlServoMasuk(status, detect1_debounced);

  // T10 — Update LCD
  // N = tt_count + tt2_count = total kendaraan di dalam area tol
  updateLCD(status, tt_fis_input, tt_count + tt2_count, CL);

  // T11 — Output Serial Monitor
  serialPrintAll();

  // T12 — Perbarui prev_detect untuk deteksi rising edge berikutnya
  prev_detect1 = detect1;
  prev_detect2 = detect2;
  prev_detect3 = detect3;

  // T13 — Timing control (kompensasi waktu eksekusi)
  long elapsed = (long)(millis() - loop_start);
  long wait    = (long)LOOP_DELAY - elapsed;
  if (wait > 0) delay((unsigned long)wait);
}
