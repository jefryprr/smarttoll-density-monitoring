#include "display.h"
#include "globals.h"

void updateLCD(int st, float tt_fis_ms, uint8_t vehicle_count, float cl_val) {
  lcd.clear();

  // ── Baris 1: Status + TT FIS Input ──
  lcd.setCursor(0, 0);
  switch (st) {
    case 0: lcd.print(F("LANCAR  ")); break;
    case 1: lcd.print(F("PADAT   ")); break;
    case 2: lcd.print(F("MACET!! ")); break;
  }
  lcd.print(F("T:"));
  lcd.print(tt_fis_ms / 1000.0f, 1);
  lcd.print(F("s"));

  // ── Baris 2: Jumlah kendaraan + CL ──
  lcd.setCursor(0, 1);
  lcd.print(F("N:"));
  lcd.print(vehicle_count);
  lcd.print(F("     CL:"));
  lcd.print(cl_val, 1);
}
