# Source Code — SmartToll Density Monitoring System

Folder ini berisi firmware ESP32 dalam **dua varian**:

## `single-tab/` — Satu file .ino (884 baris)

Semua kode dalam satu file `.ino` — cocok untuk upload langsung dari
Arduino IDE tanpa perlu setup multi-tab.

**Cara pakai:**
1. Copy `SmartToll_Density_Monitor.ino` ke folder Arduino sketch
   (misal `~/Arduino/SmartToll_Density_Monitor/`).
2. Buka di Arduino IDE, pilih board **ESP32 Dev Module**, upload.

## `multi-tab/` — Terpisah per modul (17 file .h/.cpp)

Kode dipisah per fungsi — lebih rapi untuk pengembangan/produksi.

**Cara pakai:**
1. Copy seluruh isi folder `multi-tab/` ke folder Arduino sketch.
2. Buka `SmartToll_Density_Monitor.ino` di Arduino IDE (file lain
   akan otomatis terkompilasi bersama).

> Kedua varian memiliki logika yang **identik** — pilih yang paling
> nyaman untuk workflow Anda.
