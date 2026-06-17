/**
 * display.cpp — Implementasi Fungsi LCD Display
 * SmartToll Density Monitoring System v3.2.1
 */

#include "display.h"

// Objek LCD global
static LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Buffer untuk formatting
static char line1[LCD_COLS + 1];
static char line2[LCD_COLS + 1];

// ============================================================
// Inisialisasi LCD
// ============================================================
void initLCD() {
    Wire.begin(21, 22); // SDA=21, SCL=22 (ESP32 default)
    lcd.init();
    lcd.backlight();
    lcd.clear();

    // Tampilkan splash screen
    lcd.setCursor(0, 0);
    lcd.print("SmartToll v3.2");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    delay(1000);
    lcd.clear();
}

// ============================================================
// Helper: Konversi Status ke String
// ============================================================
static const char* statusToString(StatusKemacetan st) {
    switch (st) {
        case LANCAR: return "LANCAR";
        case PADAT:  return "PADAT ";
        case MACET:  return "MACET ";
        default:     return "??????";
    }
}

// ============================================================
// Update LCD
// ============================================================
void updateLCD(StatusKemacetan st, float tt_dom, unsigned long v_count, float cl_val) {
    // Line 1: Status + TT
    // Format: "LANCAR  TT:1234"
    snprintf(line1, LCD_COLS + 1, "%s T:%4lu",
             statusToString(st),
             (unsigned long)tt_dom);

    // Line 2: Vehicle count + CL
    // Format: "N:123   CL:45.2"
    snprintf(line2, LCD_COLS + 1, "N:%-4lu CL:%4.1f",
             v_count,
             cl_val);

    // Tulis ke LCD
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}
