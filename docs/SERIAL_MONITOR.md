# Panduan Membaca Serial Monitor

Firmware mencetak log yang sangat detail ke Serial Monitor (baud rate **115200**) pada setiap siklus `loop()`. Dokumen ini menjelaskan arti setiap baris agar memudahkan debugging dan demonstrasi.

## Format Umum Satu Siklus Log

```
================================================
[LOOP #142]
  [Sensor]
    S1 (Masuk) : 8.42 cm | Det: 1 | Debounced: YA
    S2 (Tengah): 28.10 cm | Det: 0
    S3 (Keluar): 29.50 cm | Det: 0
  [Microscopic Variables]
    TT1 [S1→S2] = 1240 ms | ⏱ LIVE — 1 kendaraan di jalur S1→S2
    TT1 Buffer  : [1240ms] (1/5 slot)
    TT2 [S2→S3] = 0 ms | (idle)
    TT2 Buffer  : [] (0/5 slot)
    TT_FIS (MAX) = 1240 ms → dominan: TT1 [S1→S2]
    OT (MAX) = 0 ms (= 0.00 s)
           OT_S1=180 ms  OT_S2=0 ms  OT_S3=0 ms
  [FIS] ─── Fuzzifikasi ───
    TT_FIS(1240 ms): Cepat=0.173  Sedang=0.000  Lama=0.000
    OT(0 ms): Singkat=1.000  Sedang=0.000  Lama=0.000
  [FIS] ─── Firing Strength Agregat ───
    α_Lancar=0.173  α_Padat=0.000  α_Macet=0.000
  [FIS] ─── COG → CL* = 4.30 ───
  [Output Sistem]
    CL (Congestion Level) = 4.30
    STATUS = LANCAR
    ServoMasuk = BUKA  [Kendaraan stabil di S1 & tidak macet → izin masuk]
================================================
```

## Penjelasan Tiap Bagian

### `[LOOP #N]`
Nomor urut siklus `loop()` sejak boot. Berguna untuk menghitung perkiraan waktu nyata (`N × LOOP_DELAY ≈ waktu sejak boot`) dan mengkorelasikan event di lapangan dengan baris log tertentu.

### `[Sensor]`
Menampilkan tiga hal per sensor:
- **Jarak terukur** dalam cm (hasil konversi `pulseIn()` → cm).
- **Det** — hasil deteksi biner (`1` = ada kendaraan, sesuai `BATAS_LEBAR_JALAN`; `0` = tidak ada).
- **Debounced** (hanya S1) — status setelah melewati filter debounce 150 ms.

### `[Microscopic Variables]`
Bagian inti yang menunjukkan kondisi dual-FIFO Travel Time:

- **`TT1 [S1→S2]`** — nilai Travel Time saat ini untuk segmen pertama. Status `⏱ LIVE` artinya ada kendaraan sedang dalam segmen ini dan nilainya diperbarui real-time (PEEK); `(idle)` artinya segmen kosong/TT = 0.
- **`TT1 Buffer`** — isi mentah FIFO: daftar waktu tempuh seluruh kendaraan yang sedang berada dalam antrean segmen S1→S2 (bisa lebih dari satu jika ada beberapa kendaraan beriringan), beserta okupansi slot (`x/5`).
- **`TT2 [S2→S3]`** dan **`TT2 Buffer`** — sama seperti di atas, tapi untuk segmen kedua (BARU sejak v3.2).
- **`TT_FIS (MAX)`** — nilai `MAX(TT1, TT2)` yang sebenarnya dikirim ke FIS, beserta keterangan **segmen mana yang dominan** — sangat membantu untuk langsung tahu di mana letak masalah jika status berubah ke PADAT/MACET.
- **`OT (MAX)`** — nilai Occupancy Time gabungan (MAX dari ketiga sensor), beserta rincian okupansi mentah tiap sensor di baris berikutnya (`OT_S1`, `OT_S2`, `OT_S3`).

### `[FIS] ─── Fuzzifikasi ───`
Menampilkan derajat keanggotaan (`μ`, rentang 0.0–1.0) dari nilai TT dan OT saat ini terhadap masing-masing himpunan fuzzy (Cepat/Sedang/Lama untuk TT; Singkat/Sedang/Lama untuk OT). Berguna untuk memverifikasi apakah membership function bekerja sesuai harapan pada nilai input tertentu.

### `[FIS] ─── Firing Strength Agregat ───`
Menampilkan `α_Lancar`, `α_Padat`, `α_Macet` — hasil agregasi (MAX) dari firing strength seluruh rule yang berkontribusi ke masing-masing kelas output. Inilah nilai yang langsung menentukan bentuk fungsi yang akan didefuzzifikasi.

Jika firing strength rule R7 signifikan (`> 0.01`), akan muncul baris tambahan:
```
[FIS] ⚠ R7b aktif (α=0.840) → Diduga mogok di titik buta S2–S3! (TT2 dominan)
```
atau
```
[FIS] ⚠ R7a aktif (α=0.840) → Diduga mogok di titik buta S1–S2! (TT1 dominan)
```
Baris ini secara eksplisit memberi tahu **segmen mana** yang kemungkinan menjadi sumber masalah — sangat berguna saat demo atau debugging fisik di lapangan tanpa perlu membaca ulang nilai TT1/TT2 secara manual.

### `[FIS] ─── COG → CL* = X ───`
Hasil akhir defuzzifikasi Centre of Gravity — nilai crisp Congestion Level dalam rentang 0–100.

### `[Output Sistem]`
- **CL** — nilai akhir Congestion Level (sama dengan hasil COG di atas, ditampilkan ulang untuk ringkasan).
- **STATUS** — klasifikasi akhir: `LANCAR`, `PADAT !`, atau `MACET (PALANG DITUTUP)`.
- **ServoMasuk** — status palang beserta alasan kontekstualnya:
  - `BUKA [Kendaraan stabil di S1 & tidak macet → izin masuk]` — palang terbuka karena kondisi admission terpenuhi.
  - `BUKA [Clearance aktif, sisa ~X ms sebelum tutup]` — palang masih dalam periode tunda sebelum menutup.
  - `TUTUP [MACET → kendaraan baru DIBLOKIR]` — admission control aktif menahan kendaraan baru karena sistem MACET.
  - `TUTUP [Tidak ada kendaraan stabil / kondisi aman]` — kondisi normal, tidak ada kendaraan menunggu.

## Tips Debugging dengan Serial Monitor

1. **Memverifikasi blind spot terdeteksi:** Tempatkan objek diam di antara S2 dan S3 (tidak menyentuh kedua sensor), lalu amati `TT2 [S2→S3]` — nilainya harus terus bertambah pada setiap baris log (`⏱ LIVE`) tanpa pernah ter-reset, hingga akhirnya status berubah menjadi PADAT/MACET dan muncul peringatan `R7b aktif`.
2. **Memverifikasi tidak ada false positive:** Lewatkan kendaraan secara normal tanpa berhenti — `TT1`/`TT2` harus kembali ke `(idle)` segera setelah kendaraan melewati sensor tujuan, dan `α_Macet` harus tetap mendekati 0.
3. **Memantau kapasitas FIFO:** Jika muncul peringatan `⚠ Antrean PENUH`, berarti lebih dari 5 kendaraan berada dalam satu segmen secara bersamaan — pertimbangkan menaikkan `TT_QUEUE_CAPACITY`/`TT2_QUEUE_CAPACITY` jika skenario pengujian Anda membutuhkan antrean lebih panjang.
4. **Mengukur akurasi timing loop:** Bandingkan timestamp antar baris `[LOOP #N]` (lewat fitur timestamp Serial Monitor Arduino IDE/PlatformIO) terhadap `LOOP_DELAY` yang diharapkan (~50 ms) untuk memastikan tidak ada bottleneck (misalnya akibat sensor timeout berulang).
