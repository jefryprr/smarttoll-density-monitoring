# Changelog

Semua perubahan penting pada proyek ini didokumentasikan dalam file ini.
Format mengacu pada prinsip [Keep a Changelog](https://keepachangelog.com/).

---

## [v3.2.1] — ESP32 Port + LCD Vehicle Count

### Diubah
- **`updateLCD()`**: baris kedua LCD diganti dari menampilkan Occupancy Time (OT) menjadi **N** — jumlah kendaraan yang sedang berada di dalam area tol.
  - `N = tt_count + tt2_count` = kendaraan di segmen S1→S2 + kendaraan di segmen S2→S3.
- Signature fungsi `updateLCD()` diperbarui: parameter `float ot_ms` digantikan `uint8_t vehicle_count`.
- Pemanggilan `updateLCD()` pada `loop()` (tahap T10) diperbarui untuk mengirim `tt_count + tt2_count` sebagai argumen jumlah kendaraan.

### Porting Arduino Uno R3 → ESP32 DevKit
| Aspek | Uno R3 | ESP32 |
|---|---|---|
| Library Servo | `<Servo.h>` | `<ESP32Servo.h>` |
| I2C LCD | otomatis (A4/A5) | `Wire.begin(21, 22)` eksplisit di `setup()` |
| Serial baud rate | 9600 | 115200 |
| Pin mapping | pin digital Uno | GPIO ESP32 (lihat tabel pin di README) |
| `pulseIn()` | kompatibel | kompatibel tanpa perubahan |
| `millis()` | kompatibel | kompatibel tanpa perubahan |
| `F()` macro | kompatibel | masih valid (opsional) |
| `memset()` | kompatibel | kompatibel tanpa perubahan |

> Seluruh logika sistem (FIS, FIFO, OT, servo, LCD) **tidak diubah** dari v3.2, kecuali penambahan tampilan jumlah kendaraan (N) yang didokumentasikan di atas.

---

## [v3.2] — Dual FIFO Travel Time (Blind Spot S2→S3)

### Masalah yang Diperbaiki

**Blind Spot S2→S3 Tidak Terdeteksi (v3.1).**
Versi v3.1 hanya melacak Travel Time (TT) di segmen S1→S2. Jika kendaraan mogok atau berhenti di antara S2 dan S3 (tidak tepat di depan sensor mana pun), sistem gagal mendeteksinya karena:
- TT1 sudah selesai dihitung saat kendaraan melewati S2.
- Occupancy Time (OT) di S2 dan S3 bernilai 0 (kendaraan tidak menghalangi sensor manapun).
- Akibatnya, output FIS = LANCAR, padahal terdapat kendaraan mogok yang menghalangi jalur.

### Ditambahkan

- FIFO buffer kedua (`tt2_queue`) yang melacak Travel Time di segmen S2→S3 secara paralel dengan TT1.
- Variabel baru: `TT2_QUEUE_CAPACITY`, `tt2_queue[]`, `tt2_head`, `tt2_tail`, `tt2_count`, `travel_time2_ms`, `tt_fis_input`.
- Variabel `prev_detect3` untuk mendeteksi rising edge pada sensor S3.
- Alur kerja TT2:
  1. PUSH ke `tt2_queue` saat rising edge S2 (mulai stopwatch S2→S3) — terjadi bersamaan dengan POP TT1.
  2. POP dari `tt2_queue` saat rising edge S3 (TT2 final dihitung).
  3. PEEK `tt2_head` untuk live tracking — TT2 naik secara real-time selagi kendaraan belum sampai S3.
  4. Idle reset: TT2 = 0 jika `tt2_count == 0` dan S2 = S3 = 0.

### Strategi Gabungan: `tt_fis_input = MAX(travel_time_ms, travel_time2_ms)`

Segmen yang paling buruk menentukan input FIS. Konsekuensinya:
- Rule R7 (`TT = Lama dan OT = Singkat → MACET`) kini otomatis berlaku untuk kedua blind spot:
  - **R7a** — TT1 dominan → kendaraan mogok di S1→S2 (perilaku v3.1, dipertahankan).
  - **R7b** — TT2 dominan → kendaraan mogok di S2→S3 (kasus baru, sebelumnya tidak terdeteksi).
- 9 rule fuzzy tidak perlu diubah sama sekali — solusi ini murni penambahan input gabungan, bukan penambahan rule.

### Diubah dari v3.1 → v3.2

| # | Bagian Kode | Perubahan |
|---|---|---|
| 1 | Global | Ditambah `TT2_QUEUE_CAPACITY`, `tt2_queue[]`, `tt2_head`, `tt2_tail`, `tt2_count`, `travel_time2_ms`, `tt_fis_input` |
| 2 | Global | Ditambah `prev_detect3` |
| 3 | `updateTravelTime()` | Rising edge S2 kini juga PUSH ke `tt2_queue`; rising edge S3 POP dari `tt2_queue` + hitung TT2 final; live tracking & idle reset TT2 ditambahkan |
| 4 | `loop()` — T5 | `tt_fis_input = MAX(TT1, TT2)`, dipakai sebagai input FIS |
| 5 | `loop()` — T9/T10 | `updateLCD()` menerima `tt_fis_input` |
| 6 | `loop()` — T11/T12 | `prev_detect3` diperbarui setiap akhir siklus |
| 7 | `setup()` | Inisialisasi buffer TT2 + `prev_detect3` |
| 8 | `serialPrintAll()` | Menampilkan isi buffer TT2 dan `tt_fis_input` beserta segmen dominan |
| 9 | Komentar | Catatan R7a/R7b ditambahkan pada dokumentasi Rule Base |

### Dipertahankan dari v3.1

- TT1 FIFO Circular Buffer (S1→S2) — logika tidak berubah.
- Live TT1 tracking + ghost-vehicle eviction (mekanisme di balik R7a).
- Debounce sensor S1, state machine servo, kontrol LED/buzzer non-blocking.
- FIS Mamdani 9-rule (TT × OT), defuzzifikasi COG step-5.
- Seluruh logika Occupancy Time (OT) — tidak berubah.

---

## [v3.1] — FIFO Travel Time, Live Tracking, FIS Mamdani

### Ditambahkan
- TT1 FIFO Circular Buffer (kapasitas 5 slot) untuk melacak Travel Time per kendaraan pada segmen S1→S2.
  - PUSH saat rising edge S1, POP saat rising edge S2.
  - Live tracking (PEEK): kendaraan terdepan dalam buffer terus dipantau waktu tempuhnya secara real-time, sehingga kendaraan yang mogok di tengah segmen tetap terdeteksi (nilai TT terus naik hingga `TT_MAX_MS`).
  - Ghost-vehicle eviction: idle reset otomatis saat buffer kosong dan seluruh sensor bersih.
- Debounce software pada sensor S1 (150 ms) untuk menghindari deteksi palsu akibat noise ultrasonik.
- State machine non-blocking untuk servo palang masuk, lengkap dengan gate clearance sebelum menutup kembali.
- Kontrol LED & buzzer non-blocking berbasis `millis()` (bukan `delay()`), sehingga tidak memblokir loop utama.
- FIS Mamdani dengan 9 rule (2 input: TT & OT, masing-masing 3 himpunan fuzzy; 1 output: Congestion Level/CL dengan 3 himpunan).
- Defuzzifikasi Centre of Gravity (COG) versi diskrit dengan step 5 (21 iterasi, domain output 0–100).
- Rule R7 (`TT = Lama dan OT = Singkat → MACET`) sebagai mekanisme inti deteksi kendaraan mogok di blind spot S1→S2.

---

## [v3.0] — Versi Awal (Arduino Uno R3)

- Implementasi awal sistem monitoring kepadatan tol di atas platform Arduino Uno R3.
- 3 sensor HC-SR04, LCD 16×2 I2C, servo SG90, indikator LED + buzzer.
- Diporting penuh ke ESP32 pada rilis v3.2.1 (lihat tabel porting di atas).
