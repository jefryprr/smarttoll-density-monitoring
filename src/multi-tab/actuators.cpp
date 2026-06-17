#include "actuators.h"
#include "globals.h"
#include "config.h"

void controlIndicators(int st) {
  unsigned long now = millis();

  switch (st) {
    case 0:   // LANCAR
      digitalWrite(LED_HIJAU,  HIGH);
      digitalWrite(LED_KUNING, LOW);
      digitalWrite(LED_MERAH,  LOW);
      ledMerahState = false;
      digitalWrite(BUZZER_PIN, LOW);
      break;

    case 1:   // PADAT
      digitalWrite(LED_HIJAU,  LOW);
      digitalWrite(LED_KUNING, HIGH);
      digitalWrite(LED_MERAH,  LOW);
      ledMerahState = false;
      if (now - lastBuzzer >= 800UL) {
        lastBuzzer = now;
        digitalWrite(BUZZER_PIN, HIGH);
        delay(80);
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case 2:   // MACET
      digitalWrite(LED_HIJAU,  LOW);
      digitalWrite(LED_KUNING, LOW);
      if (now - lastBlink >= 300UL) {
        lastBlink     = now;
        ledMerahState = !ledMerahState;
        digitalWrite(LED_MERAH, ledMerahState);
      }
      if (now - lastBuzzer >= 250UL) {
        lastBuzzer = now;
        digitalWrite(BUZZER_PIN, HIGH);
        delay(40);
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;
  }
}

void controlServoMasuk(int st, bool det1_debounced) {
  unsigned long now = millis();

  bool kendaraanMenunggu = det1_debounced;
  bool kapasitasTersedia = (st != 2);

  if (kendaraanMenunggu && kapasitasTersedia) {
    servoMasuk.write(SERVO_BUKA);
    isGateOpen       = true;
    lastGateOpenTime = now;

  } else if (isGateOpen && (now - lastGateOpenTime > GATE_CLEARANCE_MS)) {
    servoMasuk.write(SERVO_TUTUP);
    isGateOpen = false;
  }
}
