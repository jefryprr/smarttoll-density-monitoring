# Panduan Wiring Hardware

Dokumen ini menjelaskan koneksi fisik seluruh komponen ke ESP32 DevKit beserta catatan praktis untuk menghindari masalah umum saat perakitan.

## Daftar Komponen

| Komponen | Jumlah |
|---|---|
| ESP32 DevKit V1 (38-pin / 30-pin) | 1 |
| Sensor Ultrasonik HC-SR04 | 3 |
| LCD 16×2 dengan modul I2C (PCF8574, alamat default `0x27`) | 1 |
| Servo SG90 | 1 |
| LED 5mm — Hijau, Kuning, Merah | 3 |
| Resistor 220Ω (untuk LED) | 3 |
| Buzzer aktif 5V | 1 |
| Breadboard ukuran penuh | 1–2 |
| Kabel jumper male-to-male & male-to-female | secukupnya |
| Power supply 5V / kabel USB | 1 |

## Tabel Koneksi Pin

### Sensor Ultrasonik HC-SR04

| Sensor | Fungsi | Pin HC-SR04 | Pin ESP32 |
|---|---|---|---|
| S1 (Zona Masuk) | Trigger | `TRIG` | GPIO 13 |
| S1 (Zona Masuk) | Echo | `ECHO` | GPIO 12 |
| S2 (Zona Tengah) | Trigger | `TRIG` | GPIO 14 |
| S2 (Zona Tengah) | Echo | `ECHO` | GPIO 27 |
| S3 (Zona Keluar) | Trigger | `TRIG` | GPIO 26 |
| S3 (Zona Keluar) | Echo | `ECHO` | GPIO 25 |
| Semua sensor | VCC | `VCC` | 5V |
| Semua sensor | GND | `GND` | GND |

> Catatan tegangan: HC-SR04 beroperasi pada 5V, namun pin ECHO mengeluarkan sinyal 5V yang melebihi batas aman GPIO ESP32 (3.3V). Untuk penggunaan jangka panjang/produksi, gunakan voltage divider (resistor 1kΩ + 2kΩ) atau level shifter pada jalur ECHO setiap sensor agar tidak merusak GPIO ESP32. Untuk prototipe singkat banyak yang berhasil tanpa divider, tapi ini tidak direkomendasikan untuk pemakaian rutin.

### Servo SG90 (Palang Masuk)

| Pin Servo | Pin ESP32 |
|---|---|
| Signal (oranye/kuning) | GPIO 5 (D5) |
| VCC (merah) | 5V (idealnya power supply eksternal, lihat catatan di bawah) |
| GND (coklat/hitam) | GND |

> Catatan arus: Servo SG90 dapat menarik arus stall hingga ~600 mA saat menahan posisi melawan beban. Jika servo jitter atau ESP32 sering brown-out reset saat servo bergerak, gunakan power supply 5V terpisah untuk servo (jangan mengandalkan pin 5V dari USB ESP32), dan tetap sambungkan GND servo ke GND ESP32 (common ground).

### LED Indikator

| LED | Anoda (+) via resistor 220Ω | Katoda (-) |
|---|---|---|
| Hijau | GPIO 18 | GND |
| Kuning | GPIO 19 | GND |
| Merah | GPIO 23 | GND |

### Buzzer Aktif

| Pin Buzzer | Pin ESP32 |
|---|---|
| Positif (+) | GPIO 4 |
| Negatif (-) | GND |

### LCD 16×2 I2C

| Pin Modul I2C | Pin ESP32 |
|---|---|
| `VCC` | 5V (atau 3.3V, sesuaikan modul) |
| `GND` | GND |
| `SDA` | GPIO 21 |
| `SCL` | GPIO 22 |

> Jika LCD tidak menyala/menampilkan kotak hitam pekat, sesuaikan potensiometer kontras pada modul I2C (biasanya trimmer kecil biru di belakang LCD).
> Alamat I2C default `0x27` digunakan di kode (`LiquidCrystal_I2C lcd(0x27, 16, 2)`). Jika modul Anda memakai alamat lain (misal `0x3F`), jalankan I2C scanner sketch terlebih dahulu dan sesuaikan alamat di kode.

## Diagram Skematik (Ringkas)

```
                                  ┌──────────────────────────┐
                                  │        ESP32 DevKit        │
                                  │                            │
   HC-SR04 S1 ── TRIG ───────────┤ GPIO13                     │
              └─ ECHO ───[divider]┤ GPIO12                     │
   HC-SR04 S2 ── TRIG ───────────┤ GPIO14                     │
              └─ ECHO ───[divider]┤ GPIO27                     │
   HC-SR04 S3 ── TRIG ───────────┤ GPIO26                     │
              └─ ECHO ───[divider]┤ GPIO25                     │
                                  │                            │
   Servo SG90 ── Signal ─────────┤ GPIO5                      │
                 (VCC dari PSU eksternal 5V, GND common)        │
                                  │                            │
   LED Hijau  ──[220Ω]───────────┤ GPIO18                     │
   LED Kuning ──[220Ω]───────────┤ GPIO19                     │
   LED Merah  ──[220Ω]───────────┤ GPIO23                     │
                                  │                            │
   Buzzer (+) ────────────────────┤ GPIO4                      │
                                  │                            │
   LCD I2C SDA ───────────────────┤ GPIO21                     │
   LCD I2C SCL ───────────────────┤ GPIO22                     │
                                  │                            │
                                  │  5V ── VCC (sensor, LCD)   │
                                  │  GND ── GND (semua)        │
                                  └──────────────────────────┘
```

## Tata Letak Fisik Sensor di Jalur Tol (Mini Track)

```
   ENTRY                                                          EXIT
     │                                                              │
  ┌──▼──┐         ┌─────────┐          ┌─────────┐          ┌─────▼─┐
  │PALANG│   S1    │  TT1    │    S2    │   TT2   │    S3    │ KELUAR │
  │MASUK ├────●────┤ segmen  ├────●─────┤ segmen  ├────●─────┤        │
  └──────┘  (sensor)└─────────┘ (sensor)└─────────┘ (sensor) └────────┘
     ▲
   Servo
   (GPIO5)
```

Jarak antar sensor disarankan minimal 30–50 cm agar terdapat ruang yang cukup untuk menguji skenario kendaraan mogok di blind spot (kendaraan diam tepat di tengah segmen, tidak menyentuh kedua sensor).

## Checklist Sebelum Menyalakan Sistem

- [ ] Seluruh GND komponen tersambung ke satu common ground (termasuk power supply eksternal servo, jika dipakai)
- [ ] Voltage divider/level shifter terpasang pada jalur ECHO ketiga sensor (jika untuk pemakaian rutin)
- [ ] Alamat I2C LCD sudah dikonfirmasi sesuai kode (`0x27`)
- [ ] Servo dalam posisi bebas bergerak penuh 0°–90° tanpa terhalang fisik palang
- [ ] Tidak ada short circuit antar pin (periksa dengan multimeter sebelum menyalakan)
- [ ] Sumber daya mencukupi — disarankan >= 1A pada rail 5V jika servo + LCD + 3 sensor aktif bersamaan
