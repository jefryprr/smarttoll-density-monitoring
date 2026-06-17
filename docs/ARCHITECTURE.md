# Arsitektur Sistem

Dokumen ini menjelaskan paradigma desain, struktur data, dan alur eksekusi internal SmartToll Density Monitoring System secara teknis.

## Daftar Isi
- [Paradigma: Discrete Event System (DES) Mikroskopis](#paradigma-discrete-event-system-des-mikroskopis)
- [Dua Variabel Inti: TT dan OT](#dua-variabel-inti-tt-dan-ot)
- [Struktur Data: Dual FIFO Circular Buffer](#struktur-data-dual-fifo-circular-buffer)
- [Alur Kejadian di Setiap Sensor](#alur-kejadian-di-setiap-sensor)
- [State Machine Servo (Admission Control)](#state-machine-servo-admission-control)
- [Kontrol Indikator Non-Blocking](#kontrol-indikator-non-blocking)
- [Alur Eksekusi `loop()` Lengkap](#alur-eksekusi-loop-lengkap)

---

## Paradigma: Discrete Event System (DES) Mikroskopis

Sebagian besar sistem deteksi kepadatan lalu lintas bersifat **makroskopis** — mengukur kepadatan secara agregat (misalnya jumlah kendaraan per satuan waktu, atau persentase okupansi ruas jalan). SmartToll mengambil pendekatan **mikroskopis**: setiap kendaraan individual dilacak sebagai *event* terpisah sejak masuk (S1) hingga keluar (S3) area pemantauan.

Pendekatan ini dipilih karena masalah blind spot — kendaraan yang berhenti di antara dua sensor — hanya dapat terdeteksi jika sistem mengetahui **berapa lama kendaraan tertentu sudah berada di dalam suatu segmen**, bukan hanya status sensor saat ini.

## Dua Variabel Inti: TT dan OT

| Variabel | Apa yang diukur | Bagaimana mendeteksi anomali |
|---|---|---|
| **Travel Time (TT)** | Waktu yang dibutuhkan satu kendaraan untuk berpindah dari satu sensor ke sensor berikutnya | Jika kendaraan mogok **di antara** dua sensor, TT terus bertambah tanpa henti (live tracking) hingga mencapai `TT_MAX_MS` — inilah yang menandakan blind spot |
| **Occupancy Time (OT)** | Berapa lama satu sensor terus-menerus terhalang oleh objek | Jika kendaraan berhenti **tepat di depan** sebuah sensor, OT sensor tersebut akan terus bertambah |

Kombinasi kedua variabel ini melalui FIS Mamdani memungkinkan sistem membedakan:
- Kendaraan **bergerak normal** (TT pendek, OT pendek)
- Kendaraan **panjang/truk yang melintas pelan** (TT pendek/sedang, OT sedang — tidak dianggap macet)
- Kendaraan **berhenti tepat di sensor** (OT lama)
- Kendaraan **mogok di blind spot** (TT lama, OT singkat — kasus yang justru paling sulit dideteksi sistem konvensional)

## Struktur Data: Dual FIFO Circular Buffer

Karena bisa terdapat lebih dari satu kendaraan dalam satu segmen secara bersamaan (misalnya antrean), TT tidak bisa disimpan sebagai satu variabel skalar — diperlukan **antrean (queue)** agar pasangan kendaraan-waktu masuk tetap konsisten dengan urutan kedatangannya (FIFO: First-In-First-Out).

```
TT1 Buffer (S1 → S2)                 TT2 Buffer (S2 → S3)
┌───┬───┬───┬───┬───┐                ┌───┬───┬───┬───┬───┐
│ t0│ t1│ t2│   │   │   5 slot       │ t0│   │   │   │   │   5 slot
└───┴───┴───┴───┴───┘                └───┴───┴───┴───┴───┘
  ↑head          ↑tail                 ↑head      ↑tail

PUSH (tail++) : timestamp masuk segmen
POP  (head++) : timestamp keluar segmen → hitung selisih = TT final
PEEK (head)   : kendaraan TERDEPAN dalam segmen → live TT
```

Kedua buffer (`tt_queue` dan `tt2_queue`) memiliki kapasitas 5 slot (`TT_QUEUE_CAPACITY` / `TT2_QUEUE_CAPACITY`), masing-masing dikelola dengan pointer `head`, `tail`, dan counter `count` — implementasi *ring buffer* klasik tanpa alokasi memori dinamis (sesuai keterbatasan microcontroller).

**Mengapa dua buffer terpisah, bukan satu?**
Karena TT1 (S1→S2) dan TT2 (S2→S3) adalah dua *segmen fisik* yang berbeda, dengan populasi kendaraan yang juga berbeda pada satu waktu. Memisahkan keduanya memastikan kendaraan yang sudah menyelesaikan TT1 tidak tercampur dengan kendaraan yang baru mulai TT2.

## Alur Kejadian di Setiap Sensor

```
                S1                      S2                      S3
                │                       │                       │
   Rising Edge  │ PUSH tt1              │ POP tt1 (TT1 final)   │
                │ (mulai TT1)           │ PUSH tt2 (mulai TT2)  │ POP tt2 (TT2 final)
                │                       │                       │
   ──────────────────── Segmen TT1 ─────┼──── Segmen TT2 ───────┼───────────────►  waktu
                                         │                       │
```

Poin penting: **rising edge di S2 memicu dua aksi sekaligus** — menyelesaikan TT1 (POP) dan memulai TT2 (PUSH). Inilah yang memastikan tidak ada *gap* waktu antara akhir segmen pertama dan awal segmen kedua; seluruh jalur S1→S3 termonitor tanpa celah.

### Live Tracking & Idle Reset

Selama suatu segmen masih memiliki kendaraan dalam antreannya (`count > 0`), nilai TT terus dihitung ulang setiap siklus loop berdasarkan kendaraan **terdepan** (`PEEK` pada `head`) — bukan menunggu sampai kendaraan tersebut keluar segmen. Ini krusial: jika menunggu POP, kendaraan yang mogok permanen tidak akan pernah memperbarui nilai TT karena POP tidak pernah terjadi.

Saat antrean kosong **dan** seluruh sensor terkait bersih (tidak ada kendaraan terdeteksi), TT direset ke 0 — mencegah nilai TT "menggantung" dari kendaraan sebelumnya (disebut juga *ghost vehicle eviction*).

## State Machine Servo (Admission Control)

```
                  ┌──────────────┐
        ┌────────►│   TERTUTUP   │◄────────┐
        │         └──────┬───────┘         │
        │                │                 │
        │   kendaraan stabil di S1          │  clearance timer habis
        │   DAN status ≠ MACET              │  (200 ms sejak buka)
        │                ▼                 │
        │         ┌──────────────┐         │
        └─────────┤    TERBUKA   ├─────────┘
                   └──────────────┘
```

Servo palang hanya terbuka jika **dua kondisi terpenuhi sekaligus**:
1. Sensor S1 mendeteksi kendaraan secara stabil (lolos debounce 150 ms).
2. Status sistem saat ini **bukan** MACET (admission control — mencegah kendaraan baru masuk ke area yang sudah padat/macet).

Setelah terbuka, servo akan tetap terbuka minimal selama `GATE_CLEARANCE_MS` (200 ms) sebelum diizinkan menutup kembali, mencegah palang "mengejut" naik-turun terlalu cepat.

## Kontrol Indikator Non-Blocking

LED dan buzzer dikendalikan tanpa fungsi `delay()` yang memblokir — semua timing diatur lewat perbandingan `millis()` terhadap timestamp terakhir (pola *non-blocking timer*). Hal ini penting agar loop utama tetap berjalan pada kecepatan konstan (`LOOP_DELAY` ≈ 50 ms) tanpa terganggu logika kedip LED atau bunyi buzzer.

| Status | LED | Buzzer |
|---|---|---|
| LANCAR (0) | Hijau menyala tetap | Mati |
| PADAT (1) | Kuning menyala tetap | Bunyi pendek setiap 800 ms |
| MACET (2) | Merah berkedip setiap 300 ms | Bunyi pendek setiap 250 ms |

## Alur Eksekusi `loop()` Lengkap

| Tahap | Nama | Deskripsi |
|---|---|---|
| T1 | Baca Sensor | Membaca dist1/dist2/dist3 secara berurutan dengan jeda anti cross-talk |
| T2 | Debounce S1 | Menstabilkan deteksi sensor masuk sebelum dianggap valid |
| T3 | Update Travel Time | Menjalankan seluruh siklus PUSH/POP/PEEK/reset untuk TT1 dan TT2 |
| T4 | Update Occupancy Time | Menghitung durasi okupansi tiap sensor, ambil nilai MAX |
| T5 | Hitung `tt_fis_input` | `MAX(travel_time_ms, travel_time2_ms)` |
| T6 | Jalankan FIS Mamdani | Fuzzifikasi → evaluasi rule → agregasi → defuzzifikasi COG → CL |
| T7 | Tentukan Status | Threshold CL → LANCAR / PADAT / MACET |
| T8 | Kontrol Indikator | LED + buzzer non-blocking sesuai status |
| T9 | Kontrol Servo | State machine admission control |
| T10 | Update LCD | Tampilkan status, TT dominan, jumlah kendaraan, CL |
| T11 | Serial Monitor | Logging detail seluruh variabel sistem |
| T12 | Update `prev_detect` | Simpan status sensor saat ini untuk deteksi rising edge siklus berikutnya |
| T13 | Timing Control | Kompensasi waktu eksekusi agar siklus loop konsisten ≈ `LOOP_DELAY` |
