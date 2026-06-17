# SmartToll Density Monitoring System (v3.2.1 — ESP32 Port)

Sistem pemantauan kepadatan jalur tol berbasis ESP32, menggunakan
**Fuzzy Inference System (Mamdani)** untuk mengklasifikasikan kondisi
lalu lintas (Lancar / Padat / Macet) dari dua variabel mikroskopis:
**Travel Time (TT)** dan **Occupancy Time (OT)**.

Proyek ini merupakan Tugas Besar mata kuliah Sistem Kendali dan
Sensor Aktuator.

## Daftar Isi

- [Arsitektur Sistem](#arsitektur-sistem) | [Detail Teknis](docs/ARCHITECTURE.md)
- [Cara Kerja](#cara-kerja)
- [Hardware & Wiring](#hardware--wiring) | [Panduan Lengkap](docs/WIRING.md)
- [Struktur Kode](#struktur-kode)
- [Cara Menjalankan](#cara-menjalankan)
- [Rule Base Fuzzy](#rule-base-fuzzy) | [Detail FIS](docs/FUZZY_LOGIC.md)
- [Serial Monitor](docs/SERIAL_MONITOR.md)
- [Changelog](#changelog)

## Arsitektur Sistem

Sistem menggunakan paradigma **Discrete Event System (DES) mikroskopis**
dengan dua input fuzzy:

| Variabel | Deskripsi | Domain |
|---|---|---|
| **TT** (Travel Time) | Waktu tempuh kendaraan antar sensor. TT panjang/timeout → kendaraan lambat atau mogok di titik buta (blind spot). | 0–10000 ms |
| **OT** (Occupancy Time) | Lama sensor terbaca HIGH kontinu (MAX dari S1/S2/S3). OT panjang → kendaraan berhenti tepat di depan sensor. | 0–5000 ms |

Output fuzzy **CL (Congestion Level)** [0–100] kemudian diklasifikasikan
menjadi status `LANCAR` / `PADAT` / `MACET`.

### Deteksi Blind Spot — Dual FIFO Travel Time

Versi v3.1 hanya melacak TT di segmen S1→S2, sehingga kendaraan yang
mogok di antara S2 dan S3 tidak terdeteksi (TT1 sudah selesai, OT
S2/S3 = 0). v3.2 menambahkan **FIFO kedua (TT2)** untuk segmen S2→S3,
sehingga:

```
tt_fis_input = MAX(TT1[S1→S2], TT2[S2→S3])
```

Segmen yang paling buruk yang menentukan output FIS — sehingga
kendaraan mogok di S1–S2 *maupun* S2–S3 terdeteksi otomatis lewat
Rule R7, tanpa perlu menambah rule baru.

## Cara Kerja

1. Tiga sensor ultrasonik (S1 zona masuk, S2 zona tengah, S3 zona
   keluar) dibaca bergantian setiap siklus loop.
2. Setiap rising edge di S1/S2/S3 memicu push/pop pada FIFO TT1
   dan TT2 (lihat `travel_time.cpp`), sekaligus melacak durasi
   okupansi per sensor (`occupancy.cpp`).
3. `tt_fis_input` (MAX TT1, TT2) dan `occupancy_time_ms` (MAX OT
   S1/S2/S3) masuk ke FIS Mamdani 9-rule (`fuzzy_logic.cpp`) →
   menghasilkan CL via defuzzifikasi Centre of Gravity (COG).
4. CL diklasifikasikan menjadi status, yang mengendalikan LED,
   buzzer, servo palang masuk, dan tampilan LCD.

## Hardware & Wiring

| Komponen | Pin ESP32 | Keterangan |
|---|---|---|
| HC-SR04 #1 (S1 — Masuk) | TRIG=13, ECHO=12 | Sebelum palang |
| HC-SR04 #2 (S2 — Tengah) | TRIG=14, ECHO=27 | Titik ukur TT1 & TT2 |
| HC-SR04 #3 (S3 — Keluar) | TRIG=26, ECHO=25 | Titik akhir TT2 |
| Servo SG90 (palang masuk) | GPIO 5 | Library `ESP32Servo` |
| LED Hijau | GPIO 18 | Status LANCAR |
| LED Kuning | GPIO 19 | Status PADAT |
| LED Merah | GPIO 23 | Status MACET (berkedip) |
| Buzzer aktif | GPIO 4 | Alarm non-blocking |
| LCD 16×2 I2C (alamat 0x27) | SDA=21, SCL=22 | Default I2C ESP32 |

> Catatan port dari Arduino Uno R3: gunakan library `ESP32Servo`
> (bukan `Servo.h`), `Wire.begin(21, 22)` wajib dipanggil di
> `setup()`, dan baud rate Serial 115200.

## Struktur Kode

Tersedia **dua varian** firmware dengan logika yang identik:

### Varian 1: Single-tab (satu file, 884 baris)

Cocok untuk upload langsung dari Arduino IDE tanpa konfigurasi tambahan.

```
src/single-tab/
└── SmartToll_Density_Monitor.ino   ← semua kode dalam satu file
```

### Varian 2: Multi-tab (terpisah per modul, 17 file)

Lebih rapi untuk development dan code review — setiap modul punya
file `.h` (header) dan `.cpp` (implementasi) terpisah.

```
src/multi-tab/
├── SmartToll_Density_Monitor.ino   orkestrasi setup() & loop()
├── config.h                        pin mapping & konstanta sistem
├── globals.h / globals.cpp         state global bersama
├── sensors.h / sensors.cpp         pembacaan HC-SR04
├── travel_time.h / travel_time.cpp dual FIFO TT1 (S1→S2) & TT2 (S2→S3)
├── occupancy.h / occupancy.cpp     durasi okupansi per sensor
├── fuzzy_logic.h / fuzzy_logic.cpp FIS Mamdani 9-rule + COG
├── actuators.h / actuators.cpp     LED, buzzer, servo
├── display.h / display.cpp         LCD 16×2 I2C
└── debug_serial.h / debug_serial.cpp  log ke Serial Monitor
```

Detail arsitektur dan alur eksekusi ada di [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Cara Menjalankan

### Opsi A: Single-tab (paling mudah)

1. Install library via Arduino Library Manager:
   - `LiquidCrystal_I2C`
   - `ESP32Servo` (oleh Kevin Harrington)
2. Copy `src/single-tab/SmartToll_Density_Monitor.ino` ke folder
   Arduino sketch baru bernama `SmartToll_Density_Monitor/`
   (folder dan file `.ino` **harus sama nama**).
3. Pilih board **ESP32 Dev Module** di Arduino IDE.
4. Sambungkan wiring sesuai tabel di atas.
5. Upload, lalu buka Serial Monitor pada baud rate **115200**.

### Opsi B: Multi-tab

1. Copy seluruh isi `src/multi-tab/` ke folder Arduino sketch
   bernama `SmartToll_Density_Monitor/`.
2. Buka `SmartToll_Density_Monitor.ino` di Arduino IDE — tab-tab
   lain (config, sensors, fuzzy_logic, dll.) akan otomatis terbuka.
3. Upload dan jalankan seperti biasa.

> Lihat [`docs/WIRING.md`](docs/WIRING.md) untuk panduan koneksi
> hardware dan checklist sebelum menyalakan sistem.

## Rule Base Fuzzy

FIS Mamdani — 9 rule, operator AND = MIN, agregasi output = MAX:

| # | TT | OT | CL | Keterangan |
|---|---|---|---|---|
| R1 | Cepat | Singkat | Lancar | Kendaraan melaju wajar |
| R2 | Cepat | Sedang | Lancar | Kendaraan panjang/truk |
| R3 | Cepat | Lama | Padat | Berhenti mendadak |
| R4 | Sedang | Singkat | Lancar | Kecepatan moderat |
| R5 | Sedang | Sedang | Padat | Mulai menumpuk |
| R6 | Sedang | Lama | Macet | Hampir berhenti |
| R7 | **Lama** | **Singkat** | **Macet** | **Deteksi blind spot** (R7a: TT1 dominan → mogok S1–S2; R7b: TT2 dominan → mogok S2–S3) |
| R8 | Lama | Sedang | Macet | Antrian terstruktur |
| R9 | Lama | Lama | Macet | Mogok di depan sensor |

Defuzzifikasi menggunakan **Centre of Gravity (COG)** diskrit dengan
step 5 (21 iterasi, domain CL 0–100).

## Changelog

**v3.2.1**
- LCD baris 2 diganti dari Occupancy Time menjadi `N` (jumlah
  kendaraan di dalam area tol = `tt_count + tt2_count`).

**v3.2**
- Menambahkan FIFO kedua (`tt2_queue`) untuk melacak Travel Time
  segmen S2→S3, menutup blind spot yang ada di v3.1.
- `tt_fis_input = MAX(TT1, TT2)` — rule base FIS tidak berubah,
  Rule R7 otomatis mencakup kedua blind spot.

**v3.1 → v3.2 (masalah yang diperbaiki)**
- v3.1 hanya melacak TT di S1→S2; kendaraan mogok di S2→S3 tidak
  terdeteksi karena TT1 sudah selesai dan OT S2/S3 = 0.

## Lisensi

Proyek ini dilisensikan di bawah [MIT License](LICENSE.md).
