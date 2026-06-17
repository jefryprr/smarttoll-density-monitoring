# Fuzzy Inference System (FIS) Mamdani — Detail Teknis

Dokumen ini menjelaskan secara rinci desain logika fuzzy yang menjadi inti pengambilan keputusan SmartToll: dari fuzzifikasi, rule base, hingga defuzzifikasi.

## Daftar Isi
- [Input & Output Sistem](#input--output-sistem)
- [Membership Function — Input TT](#membership-function--input-tt)
- [Membership Function — Input OT](#membership-function--input-ot)
- [Membership Function — Output CL](#membership-function--output-cl)
- [Rule Base Mamdani (9 Rules)](#rule-base-mamdani-9-rules)
- [Inferensi: Firing Strength & Agregasi](#inferensi-firing-strength--agregasi)
- [Defuzzifikasi: Centre of Gravity (COG)](#defuzzifikasi-centre-of-gravity-cog)
- [Klasifikasi Status Akhir](#klasifikasi-status-akhir)
- [Contoh Numerik](#contoh-numerik)
- [Panduan Kalibrasi](#panduan-kalibrasi)

---

## Input & Output Sistem

| Variabel | Tipe | Domain | Himpunan Fuzzy |
|---|---|---|---|
| **TT** (`tt_fis_input`) | Input | `[0, 10000]` ms | Cepat, Sedang, Lama |
| **OT** (`occupancy_time_ms`) | Input | `[0, 5000]` ms | Singkat, Sedang, Lama |
| **CL** (Congestion Level) | Output | `[0, 100]` (crisp) | Lancar, Padat, Macet |

`tt_fis_input` bukan TT mentah dari satu segmen, melainkan **`MAX(TT1, TT2)`** — nilai terburuk dari segmen S1→S2 dan S2→S3 (lihat [`CHANGELOG.md`](../CHANGELOG.md) bagian v3.2 untuk rasionalnya).

## Membership Function — Input TT

| Himpunan | Bentuk | Parameter |
|---|---|---|
| **Cepat** | Trapesium | `[0, 0, 1500, 3000]` |
| **Sedang** | Segitiga | `[2000, 4000, 6000]` |
| **Lama** | Trapesium | `[5000, 8000, 10000, 10000]` |

```
μ
1.0 ┤████╲          ╱▲╲           ╱████████
    │    ╲        ╱  │ ╲        ╱
    │     ╲      ╱   │  ╲      ╱
0.5 ┤      ╲    ╱    │   ╲    ╱
    │       ╲  ╱     │    ╲  ╱
0.0 ┼────────╲╱──────┼─────╲╱────────────────► TT (ms)
    0   1500 3000  4000  6000   8000  10000
       Cepat        Sedang         Lama
```

## Membership Function — Input OT

| Himpunan | Bentuk | Parameter |
|---|---|---|
| **Singkat** | Trapesium | `[0, 0, 500, 1500]` |
| **Sedang** | Segitiga | `[1000, 2000, 3000]` |
| **Lama** | Trapesium | `[2500, 4000, 5000, 5000]` |

```
μ
1.0 ┤██╲        ╱▲╲          ╱██████
    │   ╲      ╱  │ ╲       ╱
0.5 ┤    ╲    ╱   │  ╲     ╱
    │     ╲  ╱    │   ╲   ╱
0.0 ┼──────╲╱─────┼────╲ ╱──────────► OT (ms)
    0   500 1500 2000 2500  4000   5000
      Singkat      Sedang      Lama
```

## Membership Function — Output CL

| Himpunan | Bentuk | Parameter |
|---|---|---|
| **Lancar** | Trapesium | `[0, 0, 10, 40]` |
| **Padat** | Segitiga | `[20, 50, 80]` |
| **Macet** | Trapesium | `[60, 80, 100, 100]` |

```
μ
1.0 ┤██╲          ╱▲╲           ╱████
    │   ╲        ╱  │ ╲        ╱
0.5 ┤    ╲      ╱   │  ╲      ╱
    │     ╲    ╱    │   ╲    ╱
0.0 ┼──────╲──╱─────┼────╲──╱─────────► CL (0–100)
    0    10 20    50    80  100
      Lancar      Padat       Macet
```

## Rule Base Mamdani (9 Rules)

Operator AND menggunakan **MIN**, agregasi antar rule pada output yang sama menggunakan **MAX** (standar inferensi Mamdani).

| # | TT | OT | → CL | Interpretasi |
|---|---|---|---|---|
| R1 | Cepat | Singkat | **Lancar** | Kendaraan melaju wajar |
| R2 | Cepat | Sedang | **Lancar** | Kendaraan panjang/truk yang melintas pelan tapi tidak berhenti |
| R3 | Cepat | Lama | **Padat** | Berhenti mendadak meski baru tiba |
| R4 | Sedang | Singkat | **Lancar** | Kecepatan moderat, tidak ada masalah |
| R5 | Sedang | Sedang | **Padat** | Mulai menumpuk |
| R6 | Sedang | Lama | **Macet** | Hampir berhenti total |
| R7 | **Lama** | **Singkat** | **Macet** | ★ **Deteksi blind spot** — lihat catatan R7a/R7b di bawah |
| R8 | Lama | Sedang | **Macet** | Antrean terstruktur (banyak kendaraan mengantre) |
| R9 | Lama | Lama | **Macet** | Mogok tepat di depan sensor |

### Mengapa R7 Adalah Rule Paling Krusial

Kombinasi **TT = Lama** namun **OT = Singkat** secara intuitif tampak kontradiktif — bagaimana mungkin kendaraan "lama" tapi sensor tidak mendeteksi okupansi yang lama? Jawabannya: kendaraan tersebut **tidak berada tepat di depan sensor manapun**, melainkan berhenti di ruang **antara** dua sensor (blind spot). Karena `tt_fis_input = MAX(TT1, TT2)`, rule ini otomatis menangani dua skenario sekaligus:

- **R7a** — `travel_time_ms` (TT1, segmen S1→S2) yang dominan → kendaraan mogok di antara S1 dan S2.
- **R7b** — `travel_time2_ms` (TT2, segmen S2→S3) yang dominan → kendaraan mogok di antara S2 dan S3 (kasus yang baru tertangani sejak v3.2).

Firmware mencetak identifikasi mana yang aktif (R7a/R7b) ke Serial Monitor setiap kali firing strength rule R7 signifikan (`> 0.01`), memudahkan debugging fisik di lapangan.

## Inferensi: Firing Strength & Agregasi

Untuk setiap rule, firing strength (`α`) dihitung sebagai **MIN** dari derajat keanggotaan kedua input:

```
α_R1 = MIN( μ_TT_Cepat(tt),  μ_OT_Singkat(ot) )
α_R2 = MIN( μ_TT_Cepat(tt),  μ_OT_Sedang(ot)  )
α_R3 = MIN( μ_TT_Cepat(tt),  μ_OT_Lama(ot)    )
α_R4 = MIN( μ_TT_Sedang(tt), μ_OT_Singkat(ot) )
α_R5 = MIN( μ_TT_Sedang(tt), μ_OT_Sedang(ot)  )
α_R6 = MIN( μ_TT_Sedang(tt), μ_OT_Lama(ot)    )
α_R7 = MIN( μ_TT_Lama(tt),   μ_OT_Singkat(ot) )
α_R8 = MIN( μ_TT_Lama(tt),   μ_OT_Sedang(ot)  )
α_R9 = MIN( μ_TT_Lama(tt),   μ_OT_Lama(ot)    )
```

Kemudian, firing strength tiap rule yang menghasilkan kelas output yang sama digabung dengan **MAX**:

```
α_Lancar = MAX(α_R1, α_R2, α_R4)
α_Padat  = MAX(α_R3, α_R5)
α_Macet  = MAX(α_R6, α_R7, α_R8, α_R9)
```

## Defuzzifikasi: Centre of Gravity (COG)

Sistem menggunakan COG diskrit dengan **step = 5** pada domain output `[0, 100]` (21 titik evaluasi: 0, 5, 10, ..., 100):

```
            Σ [ z_i × μ_agregat(z_i) ]
   CL* =   ─────────────────────────────       , z_i = 0, 5, 10, ..., 100
                 Σ [ μ_agregat(z_i) ]
```

Untuk setiap titik `z_i`, nilai keanggotaan tiap kelas output **diklip (clipping/MIN)** terhadap firing strength rule yang sesuai, lalu **diagregasi (MAX)** antar kelas:

```
clipped_Lancar(z) = MIN( α_Lancar, μ_CL_Lancar(z) )
clipped_Padat(z)  = MIN( α_Padat,  μ_CL_Padat(z)  )
clipped_Macet(z)  = MIN( α_Macet,  μ_CL_Macet(z)  )

agregat(z) = MAX( clipped_Lancar(z), clipped_Padat(z), clipped_Macet(z) )
```

Jika seluruh `α` bernilai 0 (kasus degenerate, seharusnya tidak terjadi karena domain TT/OT saling melengkapi), sistem mengembalikan `CL = 0` untuk menghindari pembagian dengan nol.

## Klasifikasi Status Akhir

Nilai crisp `CL*` kemudian diklasifikasikan menjadi status diskrit menggunakan dua threshold:

```cpp
if      (CL < 34.0) status = LANCAR;
else if (CL < 67.0) status = PADAT;
else                 status = MACET;
```

| Threshold | Nilai | Konstanta |
|---|---|---|
| Batas Lancar→Padat | 34.0 | `CL_BATAS_LANCAR` |
| Batas Padat→Macet | 67.0 | `CL_BATAS_PADAT` |

## Contoh Numerik

**Skenario:** TT1 = 2000 ms (kendaraan normal di S1→S2), TT2 = 8500 ms (kendaraan mogok di S2→S3).

1. `tt_fis_input = MAX(2000, 8500) = 8500 ms`
2. Fuzzifikasi TT pada 8500 ms → `μ_Cepat = 0`, `μ_Sedang = 0`, `μ_Lama ≈ 1.0` (plateau trapesium Lama: 8000–10000)
3. Asumsikan OT = 200 ms (sensor S2 dan S3 bersih, tidak ada okupansi) → `μ_Singkat ≈ 1.0`, `μ_Sedang = 0`, `μ_Lama = 0`
4. `α_R7 = MIN(μ_Lama_TT, μ_Singkat_OT) = MIN(1.0, 1.0) = 1.0` → **R7b aktif kuat**
5. `α_Macet = 1.0` (didominasi R7) → defuzzifikasi COG menghasilkan `CL* ≈ 90` → **status = MACET**

Inilah skenario yang **tidak akan terdeteksi oleh sistem berbasis OT semata**, namun berhasil ditangkap oleh kombinasi TT+OT dengan strategi dual-FIFO MAX.

## Panduan Kalibrasi

Jika ingin menyesuaikan sensitivitas sistem terhadap kondisi lapangan (misalnya jarak antar sensor yang berbeda, atau profil kecepatan kendaraan yang berbeda):

1. **Ukur waktu tempuh normal** kendaraan pada jarak antar sensor Anda dalam kondisi LANCAR (tanpa hambatan) — nilai ini harus berada di zona **Cepat** pada MF TT.
2. **Ukur waktu tempuh kendaraan lambat** (misalnya truk/motor matic yang melintas pelan tanpa berhenti) — nilai ini sebaiknya jatuh di zona **Sedang**, agar tidak salah diklasifikasikan macet (lihat R2, R5).
3. **Tentukan ambang "mogok"** — durasi maksimum yang dianggap wajar sebelum suatu kendaraan dianggap bermasalah; ini menjadi batas bawah zona **Lama**.
4. Sesuaikan parameter `mf_TT_*` dan `mf_OT_*` di kode secara proporsional, lalu uji ulang dengan skenario blind spot (kendaraan diam di antara dua sensor) untuk memverifikasi R7a/R7b tetap aktif sebagaimana mestinya.
5. Threshold `CL_BATAS_LANCAR` / `CL_BATAS_PADAT` dapat disesuaikan untuk membuat sistem lebih "sensitif" (ambang lebih rendah → lebih cepat menyatakan PADAT/MACET) atau lebih "toleran" (ambang lebih tinggi).
