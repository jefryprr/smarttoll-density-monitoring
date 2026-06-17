# Panduan Kontribusi

Terima kasih atas minat Anda untuk berkontribusi pada proyek **SmartToll Density Monitoring System**! Dokumen ini menjelaskan alur kerja yang diharapkan agar kontribusi dapat diproses dengan lancar.

## Cara Berkontribusi

1. **Fork** repository ini ke akun GitHub Anda.
2. **Clone** hasil fork ke komputer lokal:
   ```bash
   git clone https://github.com/<username-anda>/smarttoll-density-monitoring.git
   ```
3. Buat **branch baru** dengan nama deskriptif:
   ```bash
   git checkout -b fitur/nama-fitur-anda
   # atau
   git checkout -b fix/nama-bug-yang-diperbaiki
   ```
4. Lakukan perubahan, lalu **commit** dengan pesan yang jelas (lihat [Konvensi Commit](#konvensi-commit) di bawah).
5. **Push** branch ke fork Anda:
   ```bash
   git push origin fitur/nama-fitur-anda
   ```
6. Buka **Pull Request (PR)** ke branch `main` repository ini, sertakan deskripsi perubahan dan alasan/motivasinya.

## Konvensi Commit

Gunakan format singkat dan deskriptif, idealnya mengikuti pola [Conventional Commits](https://www.conventionalcommits.org/):

```
<tipe>: <deskripsi singkat>

[opsional: penjelasan lebih detail]
```

Tipe yang umum digunakan:
- `feat:` — penambahan fitur baru
- `fix:` — perbaikan bug
- `docs:` — perubahan dokumentasi saja
- `refactor:` — perubahan struktur kode tanpa mengubah perilaku
- `test:` — penambahan/perbaikan pengujian
- `chore:` — perubahan tooling, konfigurasi, dependency

Contoh:
```
feat: tambahkan dukungan multi-lajur untuk segmen TT3

fix: perbaiki overflow pada perhitungan elapsed time saat millis() rollover

docs: tambahkan diagram wiring untuk modul I2C alternatif 0x3F
```

## Standar Kode

- Gunakan **bahasa Indonesia** untuk komentar dan dokumentasi (mengikuti konvensi proyek), kecuali untuk nama variabel/fungsi yang sebaiknya tetap dalam bahasa Inggris (`camelCase` untuk fungsi, `snake_case` untuk variabel sesuai pola yang sudah ada di kode).
- Pertahankan gaya pemisah blok kode (`// ════...` dan `// ╔══...╗`) yang sudah konsisten di seluruh firmware untuk keterbacaan.
- Setiap fungsi baru wajib disertai docblock `/** @brief ... */` yang menjelaskan tujuan, parameter, dan return value.
- Hindari penggunaan `delay()` yang memblokir di dalam `loop()` kecuali untuk operasi sensor yang memang membutuhkannya (`readUltrasonic`, animasi LED non-kritis saat `setup()`); gunakan pola non-blocking berbasis `millis()` untuk logika lain.
- Jika menambah parameter konfigurasi baru, tambahkan sebagai `#define` di bagian **KONSTANTA SISTEM** agar tetap mudah dikalibrasi tanpa menyentuh logika inti.

## Melaporkan Bug

Gunakan template [Bug Report](.github/ISSUE_TEMPLATE/bug_report.md) saat membuka issue. Sertakan:
- Versi firmware (lihat header `CHANGELOG.md`)
- Board yang digunakan (ESP32 DevKit V1/lainnya)
- Langkah reproduksi
- Cuplikan log Serial Monitor yang relevan (lihat [`docs/SERIAL_MONITOR.md`](docs/SERIAL_MONITOR.md) untuk cara membacanya)

## Mengajukan Fitur Baru

Gunakan template [Feature Request](.github/ISSUE_TEMPLATE/feature_request.md). Jelaskan motivasi/masalah yang ingin diselesaikan, bukan hanya solusi yang diinginkan — ini membantu diskusi menemukan pendekatan terbaik.

## Pengujian Sebelum Mengajukan PR

Karena proyek ini berbasis hardware, idealnya setiap perubahan logika inti (FIS, FIFO, debounce, state machine servo) diuji dengan skenario manual berikut sebelum PR diajukan:

1. Kendaraan melintas normal tanpa berhenti → status harus tetap LANCAR.
2. Kendaraan berhenti tepat di depan salah satu sensor → status harus naik menjadi PADAT/MACET sesuai durasi.
3. Objek diam di antara dua sensor (blind spot S1–S2 dan S2–S3) → TT segmen terkait harus terus naik dan memicu R7a/R7b.
4. Beberapa kendaraan beriringan dalam satu segmen → FIFO harus tetap konsisten (urutan PUSH/POP sesuai urutan kedatangan).
5. Servo tidak boleh terbuka saat status MACET meski ada kendaraan stabil di S1.

Sertakan cuplikan log Serial Monitor dari pengujian ini di deskripsi PR jika perubahan menyentuh logika inti.

## Kode Etik

Bersikap hormat dan konstruktif dalam diskusi, review, maupun issue. Kritik terhadap kode dipersilakan, namun selalu disertai saran perbaikan yang membangun.
