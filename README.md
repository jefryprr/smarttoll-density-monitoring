# 🛣️ SmartToll Density Monitoring System

**Sistem Monitoring Kepadatan Gerbang Tol Berbasis Fuzzy Inference System (FIS) Mamdani — ESP32**

[![Platform](https://img.shields.io/badge/platform-ESP32-blue)]()
[![Language](https://img.shields.io/badge/language-C%2B%2B%20(Arduino)-orange)]()
[![Version](https://img.shields.io/badge/version-v3.2.1-brightgreen)]()
[![License](https://img.shields.io/badge/license-MIT-lightgrey)]()

> Tugas Besar — Sistem Kendali dan Sensor Aktuator

SmartToll adalah sistem prototipe gerbang tol pintar yang memantau kepadatan lalu lintas secara **mikroskopis** (per kendaraan, bukan agregat) menggunakan tiga sensor ultrasonik, mesin logika fuzzy Mamdani, dan palang otomatis. Sistem ini dirancang khusus untuk mendeteksi kendaraan yang **mogok atau berhenti di titik buta** (blind spot) antar sensor — kondisi yang umumnya terlewat oleh sistem deteksi kepadatan konvensional.

---

## 📋 Daftar Isi

- [Latar Belakang & Masalah](#-latar-belakang--masalah)
- [Fitur Utama](#-fitur-utama)
- [Arsitektur Sistem](#-arsitektur-sistem)
- [Komponen Hardware](#-komponen-hardware)
- [Peta Pin (Pin Mapping)](#-peta-pin-pin-mapping)
- [Cara Kerja Singkat](#-cara-kerja-singkat)
- [Instalasi & Build](#-instalasi--build)
- [Konfigurasi & Kalibrasi](#-konfigurasi--kalibrasi)
- [Tampilan LCD & Serial Monitor](#-tampilan-lcd--serial-monitor)
- [Struktur Proyek](#-struktur-proyek)
- [Riwayat Versi](#-riwayat-versi)
- [Roadmap](#-roadmap)
- [Kontribusi](#-kontribusi)
- [Lisensi](#-lisensi)

---

## 🎯 Latar Belakang & Masalah

Sistem deteksi kepadatan tol konvensional umumnya hanya mengandalkan **Occupancy Time (OT)** — durasi sensor terhalang oleh kendaraan. Pendekatan ini punya kelemahan fatal:

> **Jika kendaraan mogok tepat di ANTARA dua sensor (bukan tepat di depannya), sistem tidak akan mendeteksi apa pun.** OT = 0 di kedua sensor, sistem menyimpulkan jalur LANCAR, padahal ada kendaraan diam yang menghalangi jalur.

SmartToll mengatasi masalah ini dengan paradigma **Discrete Event System (DES) mikroskopis**: setiap kendaraan dilacak individual menggunakan dua variabel gabungan:

| Variabel | Definisi | Mendeteksi |
|---|---|---|
| **Travel Time (TT)** | Waktu tempuh kendaraan antar dua sensor (FIFO per kendaraan) | Kendaraan **lambat/mogok di antara sensor** (blind spot) |
| **Occupancy Time (OT)** | Durasi sensor terus-menerus terhalang | Kendaraan **berhenti tepat di depan sensor** |

Dengan mengombinasikan kedua variabel ini lewat **Fuzzy Inference System (FIS) Mamdani 9-rule**, sistem dapat membedakan kondisi LANCAR, PADAT, dan MACET secara akurat — termasuk kasus blind spot yang selama ini tidak terdeteksi.

---

## ✨ Fitur Utama

- ✅ **Dual FIFO Travel-Time Tracking** — dua circular buffer independen melacak TT di segmen S1→S2 *dan* S2→S3 secara paralel, sehingga **tidak ada blind spot** di sepanjang jalur S1→S3.
- ✅ **Strategi MAX(TT1, TT2)** — segmen yang paling buruk menentukan input FIS, tanpa perlu menambah rule fuzzy baru.
- ✅ **FIS Mamdani 9-Rule** dengan 2 input (TT, OT) dan defuzzifikasi **Centre of Gravity (COG)** diskrit.
- ✅ **Live tracking & ghost-vehicle eviction** — TT kendaraan terdepan terus dihitung secara real-time selama kendaraan belum mencapai sensor berikutnya.
- ✅ **Debounce software** pada sensor pintu masuk (S1) untuk mencegah pembacaan palsu akibat noise ultrasonik.
- ✅ **State machine non-blocking** untuk kontrol servo palang (buka/tutup + gate clearance).
- ✅ **Indikator non-blocking** (LED tiga warna + buzzer) menggunakan `millis()`, tanpa `delay()` yang memblokir loop utama.
- ✅ **LCD 16×2 I2C** menampilkan status real-time: status kemacetan, waktu tempuh dominan, jumlah kendaraan di dalam area tol, dan Congestion Level (CL).
- ✅ **Logging Serial Monitor super detail** — tiap siklus loop mencetak status sensor, isi buffer FIFO, hasil fuzzifikasi, firing strength tiap rule, hingga identifikasi blind spot mana (R7a/R7b) yang aktif.
- ✅ Sudah diporting penuh dari **Arduino Uno R3 → ESP32 DevKit** (servo, I2C, baud rate, pin mapping).

---

## 🏗️ Arsitektur Sistem

```
                    ┌─────────────────────────────────────────┐
                    │            ESP32 DevKit (MCU)            │
                    └─────────────────────────────────────────┘
                                       │
        ┌──────────────────┬──────────┴───────────┬──────────────────┐
        │                   │                      │                  │
   ┌────▼────┐        ┌─────▼────┐           ┌─────▼─────┐     ┌──────▼──────┐
   │ HC-SR04  │        │ HC-SR04  │           │ HC-SR04   │     │ LCD 16x2 I2C │
   │  S1      │        │  S2      │           │  S3       │     │  (0x27)      │
   │ (Masuk)  │        │ (Tengah) │           │ (Keluar)  │     └──────────────┘
   └────┬─────┘        └────┬─────┘           └─────┬─────┘
        │                   │                       │
        │   TT1 segmen      │      TT2 segmen        │
        │   (S1 → S2)       │      (S2 → S3)         │
        └─────────┬─────────┴───────────┬────────────┘
                   │                    │
              ┌────▼────┐          ┌────▼─────┐
              │ FIFO TT1│          │ FIFO TT2 │   ← Dual circular buffer
              │ (5 slot)│          │ (5 slot) │
              └────┬────┘          └────┬─────┘
                   │     MAX(TT1,TT2)   │
                   └─────────┬──────────┘
                              │  tt_fis_input
                   ┌──────────▼───────────┐        ┌────────────────────┐
                   │   FIS Mamdani 9-Rule  │◄───────┤ Occupancy Time (OT) │
                   │  (TT × OT → CL)       │        │ MAX(OT_S1,S2,S3)    │
                   └──────────┬───────────┘        └────────────────────┘
                              │  CL (Congestion Level, 0–100)
                   ┌──────────▼───────────┐
                   │  Klasifikasi Status   │
                   │ LANCAR / PADAT / MACET│
                   └──────────┬───────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
  ┌─────▼─────┐        ┌──────▼──────┐       ┌──────▼──────┐
  │ LED + Buzzer│       │ Servo Palang │       │  LCD + Serial│
  │ (indikator) │       │ (admission)  │       │  (monitoring)│
  └─────────────┘       └─────────────┘       └─────────────┘
```

Penjelasan arsitektur lebih lengkap (termasuk diagram alur FIFO dan diagram state machine servo) tersedia di [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## 🔧 Komponen Hardware

| No | Komponen | Jumlah | Fungsi |
|----|----------|--------|--------|
| 1 | ESP32 DevKit V1 | 1 | Microcontroller utama |
| 2 | Sensor Ultrasonik HC-SR04 | 3 | Deteksi kendaraan di zona masuk (S1), tengah (S2), keluar (S3) |
| 3 | LCD 16×2 I2C (alamat `0x27`) | 1 | Tampilan status real-time |
| 4 | Servo SG90 | 1 | Palang/portal masuk gerbang tol |
| 5 | LED 5mm (Hijau, Kuning, Merah) | 3 | Indikator status LANCAR/PADAT/MACET |
| 6 | Buzzer aktif | 1 | Alarm suara untuk status PADAT/MACET |
| 7 | Breadboard + kabel jumper | secukupnya | Wiring prototipe |
| 8 | Power supply 5V (USB / adaptor) | 1 | Sumber daya ESP32 + servo |

> 💡 Detail wiring lengkap (diagram pin & catatan praktis) ada di [`docs/WIRING.md`](docs/WIRING.md).

---

## 📌 Peta Pin (Pin Mapping)

| Komponen | Pin ESP32 | Keterangan |
|----------|-----------|------------|
| Sensor S1 — TRIG | GPIO 13 | Zona masuk (tepat sebelum palang) |
| Sensor S1 — ECHO | GPIO 12 | |
| Sensor S2 — TRIG | GPIO 14 | Zona tengah (titik ukur TT1 selesai & TT2 mulai) |
| Sensor S2 — ECHO | GPIO 27 | |
| Sensor S3 — TRIG | GPIO 26 | Zona keluar (titik ukur TT2 selesai) |
| Sensor S3 — ECHO | GPIO 25 | |
| Servo Palang Masuk | GPIO 5 (D5) | PWM via `ESP32Servo` |
| LED Hijau | GPIO 18 | Status LANCAR |
| LED Kuning | GPIO 19 | Status PADAT |
| LED Merah | GPIO 23 | Status MACET (berkedip) |
| Buzzer Aktif | GPIO 4 | Alarm suara |
| LCD I2C — SDA | GPIO 21 | Default I2C ESP32 |
| LCD I2C — SCL | GPIO 22 | Default I2C ESP32 |

---

## ⚙️ Cara Kerja Singkat

1. **Pembacaan sensor** — Ketiga sensor HC-SR04 dibaca bergantian (time-multiplexed) tiap siklus loop dengan jeda 15 ms untuk mencegah cross-talk akustik.
2. **Debounce S1** — Status sensor masuk di-debounce 150 ms agar palang tidak terbuka akibat pembacaan sesaat/noise.
3. **Dual FIFO Travel Time:**
   - **TT1 (S1→S2):** *push* saat rising edge S1, *pop* + hitung waktu tempuh final saat rising edge S2.
   - **TT2 (S2→S3):** *push* saat rising edge S2 (bersamaan dengan pop TT1), *pop* + hitung waktu tempuh final saat rising edge S3.
   - Selama kendaraan belum mencapai sensor tujuan, TT terus di-*live update* (PEEK) — sehingga kendaraan mogok di blind spot tetap terdeteksi karena TT terus naik tanpa batas hingga `TT_MAX_MS`.
4. **Occupancy Time** — Durasi setiap sensor terhalang kontinu dihitung, lalu diambil nilai **MAX** dari ketiga sensor sebagai input OT ke FIS.
5. **Gabungan TT** — `tt_fis_input = MAX(TT1, TT2)`: segmen mana pun yang terburuk akan menentukan keputusan FIS.
6. **FIS Mamdani 9-Rule** — TT dan OT difuzzifikasi (Cepat/Sedang/Lama × Singkat/Sedang/Lama), dievaluasi lewat 9 rule (AND = MIN, agregasi = MAX), lalu didefuzzifikasi dengan **Centre of Gravity (COG)** diskrit (step 5, domain 0–100) menjadi nilai crisp **Congestion Level (CL)**.
7. **Klasifikasi status** — `CL < 34` → **LANCAR**, `34 ≤ CL < 67` → **PADAT**, `CL ≥ 67` → **MACET**.
8. **Aktuasi** — LED & buzzer menyala sesuai status (non-blocking), servo palang terbuka hanya jika ada kendaraan stabil di S1 **dan** status bukan MACET (admission control), LCD & Serial Monitor menampilkan seluruh state sistem.

Detail lengkap rule base, membership function, dan proses defuzzifikasi ada di [`docs/FUZZY_LOGIC.md`](docs/FUZZY_LOGIC.md).

---

## 🚀 Instalasi & Build

### Prasyarat

- [Arduino IDE](https://www.arduino.cc/en/software) (≥ 1.8.19) atau [PlatformIO](https://platformio.org/)
- [Board Manager **ESP32 by Espressif Systems**](https://github.com/espressif/arduino-esp32) terpasang di Arduino IDE
- Library berikut terpasang via Library Manager:
  - `ESP32Servo` (by Kevin Harrington / John K. Bennett)
  - `LiquidCrystal_I2C` (by Frank de Brabander / Marco Schwartz)
  - `Wire` (built-in core ESP32)

### Langkah Instalasi

```bash
# 1. Clone repository
git clone https://github.com/<username>/smarttoll-density-monitoring.git
cd smarttoll-density-monitoring

# 2. Buka file sumber di Arduino IDE
#    src/SmartToll_v3.2.1.ino

# 3. Pilih board: Tools → Board → ESP32 Arduino → ESP32 Dev Module

# 4. Pilih port serial yang sesuai, lalu Upload (Ctrl+U)

# 5. Buka Serial Monitor dengan baud rate 115200 untuk melihat log sistem
```

### Build dengan PlatformIO (opsional)

Buat `platformio.ini` seperti berikut di root proyek:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    madhephaestus/ESP32Servo
    marcoschwartz/LiquidCrystal_I2C
```

```bash
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

---

## 🎛️ Konfigurasi & Kalibrasi

Seluruh parameter sistem didefinisikan sebagai konstanta di bagian atas `SmartToll_v3.2.1.ino`, sehingga mudah dikalibrasi tanpa mengubah logika inti:

| Konstanta | Nilai Default | Keterangan |
|---|---|---|
| `BATAS_LEBAR_JALAN` | 12.0 cm | Ambang batas jarak untuk menyatakan "ada kendaraan" |
| `LOOP_DELAY` | 50 ms | Target durasi satu siklus loop |
| `SENSOR_DELAY` | 15 ms | Jeda antar pembacaan sensor (anti cross-talk) |
| `DEBOUNCE_MS` | 150 ms | Waktu stabilisasi sensor S1 sebelum dianggap valid |
| `GATE_CLEARANCE_MS` | 200 ms | Waktu tunda sebelum palang menutup kembali |
| `TT_QUEUE_CAPACITY` / `TT2_QUEUE_CAPACITY` | 5 slot | Kapasitas FIFO — jumlah kendaraan maksimum yang dapat dilacak bersamaan per segmen |
| `TT_MAX_MS` | 10000 ms | Batas atas domain fuzzy Travel Time |
| `OT_MAX_MS` | 5000 ms | Batas atas domain fuzzy Occupancy Time |
| `CL_BATAS_LANCAR` | 34.0 | Ambang CL untuk status LANCAR → PADAT |
| `CL_BATAS_PADAT` | 67.0 | Ambang CL untuk status PADAT → MACET |
| `SERVO_BUKA` / `SERVO_TUTUP` | 90° / 0° | Sudut servo terbuka/tertutup |

> ⚠️ Mengubah domain fuzzy (`TT_MAX_MS`, `OT_MAX_MS`) memerlukan penyesuaian ulang parameter membership function di bagian *MF INPUT*. Lihat [`docs/FUZZY_LOGIC.md`](docs/FUZZY_LOGIC.md) untuk panduan kalibrasi.

---

## 🖥️ Tampilan LCD & Serial Monitor

**LCD 16×2:**
```
┌────────────────┐
│LANCAR  T: 2.8s │   ← Status + Travel Time dominan (MAX TT1, TT2)
│N:3     CL:12.5 │   ← Jumlah kendaraan di dalam area tol + Congestion Level
└────────────────┘
```

**Serial Monitor (ringkasan tiap siklus):**
```
================================================
[LOOP #142]
  [Sensor]
    S1 (Masuk) : 8.42 cm | Det: 1 | Debounced: YA
    S2 (Tengah): 28.10 cm | Det: 0
    S3 (Keluar): 29.50 cm | Det: 0
  [Microscopic Variables]
    TT1 [S1→S2] = 1240 ms | ⏱ LIVE — 1 kendaraan di jalur S1→S2
    TT2 [S2→S3] = 0 ms | (idle)
    TT_FIS (MAX) = 1240 ms → dominan: TT1 [S1→S2]
    OT (MAX) = 0 ms (= 0.00 s)
  [Output Sistem]
    CL (Congestion Level) = 8.30
    STATUS = LANCAR
    ServoMasuk = BUKA  [Kendaraan stabil di S1 & tidak macet → izin masuk]
================================================
```

Format lengkap dan penjelasan tiap baris log ada di [`docs/SERIAL_MONITOR.md`](docs/SERIAL_MONITOR.md).

---

## 📁 Struktur Proyek

```
smarttoll-density-monitoring/
├── README.md                   ← Dokumen ini
├── CHANGELOG.md                ← Riwayat versi v3.0 → v3.2.1
├── CONTRIBUTING.md             ← Panduan kontribusi
├── LICENSE.md                  ← Lisensi MIT
├── .gitignore                  ← Ignore rules untuk Arduino/PlatformIO
├── docs/
│   ├── ARCHITECTURE.md         ← Detail arsitektur, DES, dual FIFO
│   ├── WIRING.md               ← Diagram & panduan wiring hardware
│   ├── FUZZY_LOGIC.md          ← Rule base, membership function, COG
│   └── SERIAL_MONITOR.md       ← Panduan membaca log Serial Monitor
├── src/
│   └── SmartToll_v3.2.1.ino    ← Firmware utama (ESP32)
└── .github/
    └── ISSUE_TEMPLATE/
        ├── bug_report.md
        └── feature_request.md
```

---

## 🕒 Riwayat Versi

| Versi | Perubahan Utama |
|---|---|
| **v3.2.1** | LCD baris 2 diganti dari Occupancy Time → jumlah kendaraan di dalam area tol (`N = tt_count + tt2_count`) |
| **v3.2** | Penambahan **Dual FIFO TT2 (S2→S3)** untuk menutup blind spot kedua; input FIS menjadi `MAX(TT1, TT2)` |
| **v3.1** | FIFO TT1 (S1→S2), live tracking, ghost-vehicle eviction, FIS Mamdani 9-rule, COG step-5 |
| **v3.0 (Uno)** | Versi awal di atas Arduino Uno R3, diporting penuh ke ESP32 di v3.2.1 |

Lihat detail teknis lengkap tiap perubahan di [`CHANGELOG.md`](CHANGELOG.md).

---

## 🗺️ Roadmap

- [ ] Logging data ke SD card / cloud (Firebase, ThingsBoard, dsb.) untuk analitik historis
- [ ] Dashboard web real-time via WiFi/MQTT (ESP32 sudah mendukung WiFi secara native)
- [ ] Dukungan multi-lajur (lebih dari satu jalur S1→S2→S3 paralel)
- [ ] Kalibrasi otomatis ambang fuzzy berdasarkan data historis (self-tuning FIS)
- [ ] Unit test untuk fungsi membership function & defuzzifikasi (PlatformIO + Unity)
- [ ] Mode simulasi tanpa hardware (mock sensor) untuk pengujian logika FIS secara terpisah

---

## 🤝 Kontribusi

Kontribusi sangat terbuka! Silakan baca [`CONTRIBUTING.md`](CONTRIBUTING.md) untuk panduan alur kerja, konvensi commit, dan cara melaporkan bug/mengajukan fitur baru.

---

## 📄 Lisensi

Proyek ini dilisensikan di bawah [MIT License](LICENSE.md) — bebas digunakan, dimodifikasi, dan didistribusikan untuk keperluan akademik maupun non-akademik dengan tetap menyertakan atribusi.

---

<p align="center">
  Dibuat untuk Tugas Besar Sistem Kendali dan Sensor Aktuator — SmartToll Density Monitoring System
</p>
