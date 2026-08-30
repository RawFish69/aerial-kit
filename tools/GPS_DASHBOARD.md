## GPS Live Dashboard

Streamlit dashboard for the GPS module. Firmware lives in
[`../firmware/gps/`](../firmware/gps/); this is the host-side viewer.

### 1) Install Python deps

```bash
cd tools
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements-gps-dashboard.txt
```

### 2) Upload firmware to ESP32

```bash
pio run -d firmware/gps -e gps_auto -t upload
```

(from the repo root; or `~/.platformio/penv/bin/pio` if `pio` is not on your PATH)

### 3) Start dashboard

```bash
.venv/bin/streamlit run gps_dashboard.py -- --port /dev/ttyACM0 --baud 115200
```

Then open the local URL shown by Streamlit (usually `http://localhost:8501`).

### Notes

- If upload fails with lock errors, close any open monitor first.
- If no coordinates appear, wait for GPS fix (clear sky helps).
- Dashboard parses lines printed by `firmware/gps/src/main.cpp` (`Fix`, `Satellites`, `Location`, `Altitude`, `Speed`).
