/**
 * display.h — Deklarasi Fungsi LCD Display
 * SmartToll Density Monitoring System v3.2.1
 * 
 * Mengelola tampilan LCD I2C 16x2.
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"
#include "globals.h"

/**
 * Menginisialisasi LCD I2C.
 * - Wire.begin(SDA=21, SCL=22)
 * - LiquidCrystal_I2C(0x27, 16, 2)
 * - Backlight ON, clear display
 */
void initLCD();

/**
 * Memperbarui tampilan LCD.
 * Line 1: Status + TT dominan (ms)
 * Line 2: Vehicle count + CL
 * 
 * @param st             Status kemacetan
 * @param tt_dom         TT dominan (ms)
 * @param v_count        Jumlah kendaraan
 * @param cl_val         Congestion Level
 */
void updateLCD(StatusKemacetan st, float tt_dom, unsigned long v_count, float cl_val);

#endif // DISPLAY_H
