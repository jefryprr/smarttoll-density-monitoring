#include "globals.h"

// ================================================================
//  DEFINISI AKTUAL VARIABEL & OBJEK GLOBAL
//  (deklarasi/extern-nya ada di globals.h)
// ================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servoMasuk;

float dist1 = 0.0f;
float dist2 = 0.0f;
float dist3 = 0.0f;

int detect1 = 0;
int detect2 = 0;
int detect3 = 0;

int prev_detect1 = 0;
int prev_detect2 = 0;
int prev_detect3 = 0;

unsigned long detect1_stable_since = 0;
bool          detect1_debounced    = false;

unsigned long tt_queue[TT_QUEUE_CAPACITY];
uint8_t       tt_head  = 0;
uint8_t       tt_tail  = 0;
uint8_t       tt_count = 0;
float         travel_time_ms = 0.0f;

unsigned long tt2_queue[TT2_QUEUE_CAPACITY];
uint8_t       tt2_head  = 0;
uint8_t       tt2_tail  = 0;
uint8_t       tt2_count = 0;
float         travel_time2_ms = 0.0f;

float tt_fis_input = 0.0f;

unsigned long occ1_start_ms = 0;
unsigned long occ2_start_ms = 0;
unsigned long occ3_start_ms = 0;
bool          occ1_active   = false;
bool          occ2_active   = false;
bool          occ3_active   = false;
float         occupancy_time_ms = 0.0f;

float CL     = 0.0f;
int   status = 0;

unsigned long lastGateOpenTime = 0;
bool          isGateOpen       = false;

unsigned long lastBlink     = 0;
bool          ledMerahState = false;
unsigned long lastBuzzer    = 0;

unsigned long loopCount = 0;
