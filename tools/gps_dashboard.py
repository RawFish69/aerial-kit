"""
Usage:
python3 -m venv .venv
.venv/bin/pip install -r requirements-gps-dashboard.txt
.venv/bin/streamlit run gps_dashboard.py -- --port /dev/ttyACM0 --baud 115200
"""

import argparse
import math
import queue
import re
import threading
import time
from dataclasses import dataclass

import pandas as pd
import pydeck as pdk
import serial
import streamlit as st


@dataclass
class GPSState:
    connected: bool = False
    fix: bool | None = None
    satellites: int | None = None
    latitude: float | None = None
    longitude: float | None = None
    latitude_kf: float | None = None
    longitude_kf: float | None = None
    altitude_m: float | None = None
    speed_knots: float | None = None
    fix_quality: int | None = None
    fix_3d: int | None = None
    pdop: float | None = None
    hdop: float | None = None
    vdop: float | None = None
    sats_in_view: int | None = None
    snr_avg: float | None = None
    snr_max: int | None = None
    snr_samples: int | None = None
    snr_max_ever: int | None = None
    snr_bars: list[int] | None = None
    pre_fix_score: int | None = None
    pre_fix_stage: str | None = None
    last_line: str = ""
    last_update_unix: float = 0.0
    last_fix_update_unix: float = 0.0
    last_sat_update_unix: float = 0.0
    last_loc_update_unix: float = 0.0
    total_lines: int = 0


FIX_RE = re.compile(r"^Fix:\s*(yes|no)\s*$", re.IGNORECASE)
SAT_RE = re.compile(r"^Satellites:\s*(\d+)\s*$", re.IGNORECASE)
LOC_RE = re.compile(
    r"^Location:\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE,
)
ALT_RE = re.compile(r"^Altitude:\s*([+-]?\d+(?:\.\d+)?)\s*m\s*$", re.IGNORECASE)
SPD_RE = re.compile(r"^Speed:\s*([+-]?\d+(?:\.\d+)?)\s*$", re.IGNORECASE)
FIXQ_RE = re.compile(r"^Fix Quality:\s*(\d+)\s*$", re.IGNORECASE)
FIX3D_RE = re.compile(r"^Fix 3D:\s*(\d+)\s*$", re.IGNORECASE)
DOP_RE = re.compile(
    r"^DOP:\s*P=([+-]?\d+(?:\.\d+)?)\s*H=([+-]?\d+(?:\.\d+)?)\s*V=([+-]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE,
)
VIEW_RE = re.compile(r"^Sats In View:\s*(\d+)\s*$", re.IGNORECASE)
SNR_RE = re.compile(
    r"^SNR:\s*avg=([+-]?\d+(?:\.\d+)?)\s*max=(\d+)\s*samples=(\d+)\s*max_ever=(\d+)\s*$",
    re.IGNORECASE,
)
SNR_BARS_RE = re.compile(r"^SNR Bars:\s*(.+)\s*$", re.IGNORECASE)
PREFIX_SCORE_RE = re.compile(r"^Pre-Fix Score:\s*(\d+)\s*/\s*100\s*$", re.IGNORECASE)
PREFIX_STAGE_RE = re.compile(r"^Pre-Fix Stage:\s*(.+)\s*$", re.IGNORECASE)
TRANSIENT_READ_ERROR_TOKEN = "device reports readiness to read but returned no data"


def fmt_age(ts: float) -> str:
    if not ts:
        return "-"
    return f"{time.time() - ts:.1f}s"


def fmt_clock(ts: float) -> str:
    if not ts:
        return "-"
    return time.strftime("%H:%M:%S", time.localtime(ts))


def quality_band(hdop: float | None, snr_avg: float | None) -> str:
    if hdop is None and snr_avg is None:
        return "Unknown"
    if hdop is not None and hdop <= 1.5 and (snr_avg is None or snr_avg >= 25):
        return "Excellent"
    if hdop is not None and hdop <= 2.5 and (snr_avg is None or snr_avg >= 18):
        return "Good"
    if hdop is not None and hdop <= 5.0:
        return "Fair"
    return "Poor"


def kalman_1d_update(measurement: float, estimate: float, cov: float, q: float, r: float) -> tuple[float, float]:
    pred_est = estimate
    pred_cov = cov + q
    k = pred_cov / (pred_cov + r)
    next_est = pred_est + k * (measurement - pred_est)
    next_cov = (1.0 - k) * pred_cov
    return next_est, max(next_cov, 1e-12)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * r * math.asin(math.sqrt(max(0.0, min(1.0, a))))


def update_kalman_location(
    lat: float, lon: float, enabled: bool, q: float, r: float
) -> tuple[float, float]:
    if not enabled:
        st.session_state.kf_initialized = False
        return lat, lon

    if not st.session_state.kf_initialized:
        st.session_state.kf_lat_est = lat
        st.session_state.kf_lon_est = lon
        st.session_state.kf_lat_cov = max(r, 1e-8)
        st.session_state.kf_lon_cov = max(r, 1e-8)
        st.session_state.kf_initialized = True
        return lat, lon

    lat_est, lat_cov = kalman_1d_update(
        measurement=lat,
        estimate=st.session_state.kf_lat_est,
        cov=st.session_state.kf_lat_cov,
        q=q,
        r=r,
    )
    lon_est, lon_cov = kalman_1d_update(
        measurement=lon,
        estimate=st.session_state.kf_lon_est,
        cov=st.session_state.kf_lon_cov,
        q=q,
        r=r,
    )

    st.session_state.kf_lat_est = lat_est
    st.session_state.kf_lon_est = lon_est
    st.session_state.kf_lat_cov = lat_cov
    st.session_state.kf_lon_cov = lon_cov
    return lat_est, lon_est


def recent_rows(rows: list[dict], window_sec: float) -> list[dict]:
    if not rows or window_sec <= 0:
        return rows
    cutoff = time.time() - window_sec
    return [r for r in rows if r["t"] >= cutoff]


def decimate_rows(rows: list[dict], max_points: int) -> list[dict]:
    if not rows or len(rows) <= max_points:
        return rows
    step = max(1, len(rows) // max_points)
    return rows[::step][-max_points:]


def parse_line(line: str, state: GPSState) -> str:
    line = line.strip()
    if not line:
        return "empty"

    state.last_line = line
    now = time.time()
    state.last_update_unix = now
    state.total_lines += 1

    m = FIX_RE.match(line)
    if m:
        state.fix = m.group(1).lower() == "yes"
        state.last_fix_update_unix = now
        return "fix"

    m = SAT_RE.match(line)
    if m:
        state.satellites = int(m.group(1))
        state.last_sat_update_unix = now
        return "sat"

    m = LOC_RE.match(line)
    if m:
        state.latitude = float(m.group(1))
        state.longitude = float(m.group(2))
        state.last_loc_update_unix = now
        return "loc"

    m = ALT_RE.match(line)
    if m:
        state.altitude_m = float(m.group(1))
        return "alt"

    m = SPD_RE.match(line)
    if m:
        state.speed_knots = float(m.group(1))
        return "spd"

    m = FIXQ_RE.match(line)
    if m:
        state.fix_quality = int(m.group(1))
        return "fixq"

    m = FIX3D_RE.match(line)
    if m:
        state.fix_3d = int(m.group(1))
        return "fix3d"

    m = DOP_RE.match(line)
    if m:
        state.pdop = float(m.group(1))
        state.hdop = float(m.group(2))
        state.vdop = float(m.group(3))
        return "dop"

    m = VIEW_RE.match(line)
    if m:
        state.sats_in_view = int(m.group(1))
        return "view"

    m = SNR_RE.match(line)
    if m:
        state.snr_avg = float(m.group(1))
        state.snr_max = int(m.group(2))
        state.snr_samples = int(m.group(3))
        state.snr_max_ever = int(m.group(4))
        return "snr"

    m = SNR_BARS_RE.match(line)
    if m:
        raw = m.group(1).strip()
        if raw.lower() == "none" or not raw:
            state.snr_bars = []
        else:
            vals = []
            for token in raw.split(","):
                token = token.strip()
                if token.isdigit():
                    vals.append(int(token))
            state.snr_bars = vals
        return "snr_bars"

    m = PREFIX_SCORE_RE.match(line)
    if m:
        state.pre_fix_score = int(m.group(1))
        return "pre_score"

    m = PREFIX_STAGE_RE.match(line)
    if m:
        state.pre_fix_stage = m.group(1).strip()
        return "pre_stage"

    return "other"


def serial_worker(port: str, baud: int, line_queue: queue.Queue[str], stop_evt: threading.Event) -> None:
    while not stop_evt.is_set():
        try:
            with serial.Serial(port=port, baudrate=baud, timeout=1, exclusive=True) as ser:
                line_queue.put("__CONNECTED__")
                while not stop_evt.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    text = raw.decode(errors="replace").strip("\x00").strip()
                    if text:
                        line_queue.put(text)
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if TRANSIENT_READ_ERROR_TOKEN in msg.lower():
                line_queue.put(f"__WARN_TRANSIENT__:{msg}")
            else:
                line_queue.put(f"__ERROR__:{msg}")
            time.sleep(1.5)


def init_session() -> None:
    defaults = {
        "state": GPSState(),
        "trail_raw": [],
        "trail_kf": [],
        "queue": queue.Queue(),
        "stop_evt": threading.Event(),
        "reader_thread": None,
        "connected_once": False,
        "raw_lines": [],
        "parse_counts": default_parse_counts(),
        "line_times": [],
        "sat_history": [],
        "fix_history": [],
        "view_history": [],
        "snr_history": [],
        "connect_count": 0,
        "error_count": 0,
        "transient_count": 0,
        "last_error": "",
        "last_warning": "",
        "last_warning_unix": 0.0,
        "kalman_enabled": True,
        "kf_q": 1e-11,
        "kf_r": 5e-7,
        "kf_initialized": False,
        "kf_lat_est": 0.0,
        "kf_lon_est": 0.0,
        "kf_lat_cov": 1.0,
        "kf_lon_cov": 1.0,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def default_parse_counts() -> dict[str, int]:
    return {
        "fix": 0,
        "sat": 0,
        "loc": 0,
        "alt": 0,
        "spd": 0,
        "fixq": 0,
        "fix3d": 0,
        "dop": 0,
        "view": 0,
        "snr": 0,
        "snr_bars": 0,
        "pre_score": 0,
        "pre_stage": 0,
        "other": 0,
    }


def clear_runtime_data(clear_state: bool) -> None:
    st.session_state.trail_raw = []
    st.session_state.trail_kf = []
    st.session_state.raw_lines = []
    st.session_state.line_times = []
    st.session_state.sat_history = []
    st.session_state.fix_history = []
    st.session_state.view_history = []
    st.session_state.snr_history = []
    st.session_state.parse_counts = default_parse_counts()
    st.session_state.last_error = ""
    st.session_state.kf_initialized = False
    if clear_state:
        was_connected = st.session_state.state.connected
        st.session_state.state = GPSState(connected=was_connected)


def prune_runtime_data(raw_log_limit: int, trail_limit: int, history_limit: int) -> None:
    st.session_state.raw_lines = st.session_state.raw_lines[-raw_log_limit:]
    st.session_state.trail_raw = st.session_state.trail_raw[-trail_limit:]
    st.session_state.trail_kf = st.session_state.trail_kf[-trail_limit:]
    for key in ("sat_history", "fix_history", "view_history", "snr_history"):
        st.session_state[key] = st.session_state[key][-history_limit:]
    st.session_state.line_times = st.session_state.line_times[-5000:]


def process_incoming(raw_log_limit: int, trail_limit: int, kalman_enabled: bool, kf_q: float, kf_r: float) -> None:
    while True:
        try:
            line = st.session_state.queue.get_nowait()
        except queue.Empty:
            break

        if line == "__CONNECTED__":
            st.session_state.state.connected = True
            st.session_state.connect_count += 1
            st.session_state.last_error = ""
            st.session_state.last_warning = ""
            st.session_state.last_warning_unix = 0.0
            st.session_state.raw_lines.append(f"[{time.strftime('%H:%M:%S')}] __CONNECTED__")
            continue

        if line.startswith("__WARN_TRANSIENT__:"):
            st.session_state.state.connected = False
            st.session_state.transient_count += 1
            st.session_state.last_warning = line.removeprefix("__WARN_TRANSIENT__:")
            st.session_state.last_warning_unix = time.time()
            st.session_state.state.last_line = st.session_state.last_warning
            st.session_state.state.last_update_unix = st.session_state.last_warning_unix
            st.session_state.raw_lines.append(
                f"[{time.strftime('%H:%M:%S')}] __WARN__ {st.session_state.last_warning}"
            )
            continue

        if line.startswith("__ERROR__:"):
            st.session_state.state.connected = False
            st.session_state.state.last_line = line.removeprefix("__ERROR__:")
            st.session_state.state.last_update_unix = time.time()
            st.session_state.error_count += 1
            st.session_state.last_error = st.session_state.state.last_line
            st.session_state.kf_initialized = False
            st.session_state.raw_lines.append(
                f"[{time.strftime('%H:%M:%S')}] __ERROR__ {st.session_state.state.last_line}"
            )
            continue

        now = time.time()
        st.session_state.line_times.append(now)
        if len(st.session_state.line_times) > 5000:
            st.session_state.line_times = st.session_state.line_times[-5000:]

        kind = parse_line(line, st.session_state.state)
        st.session_state.parse_counts[kind] = st.session_state.parse_counts.get(kind, 0) + 1

        st.session_state.raw_lines.append(f"[{time.strftime('%H:%M:%S')}] {line}")
        if len(st.session_state.raw_lines) > raw_log_limit:
            st.session_state.raw_lines = st.session_state.raw_lines[-raw_log_limit:]

        s = st.session_state.state
        if kind == "sat":
            st.session_state.sat_history.append({"t": now, "sat": s.satellites or 0})
        elif kind == "fix":
            st.session_state.fix_history.append({"t": now, "fix": 1 if s.fix else 0})
        elif kind == "view":
            st.session_state.view_history.append({"t": now, "view": s.sats_in_view or 0})
        elif kind == "snr":
            st.session_state.snr_history.append({"t": now, "snr_avg": s.snr_avg or 0.0})

        for key in ("sat_history", "fix_history", "view_history", "snr_history"):
            if len(st.session_state[key]) > 2000:
                st.session_state[key] = st.session_state[key][-2000:]

        if s.latitude is not None and s.longitude is not None:
            lat_kf, lon_kf = update_kalman_location(
                lat=s.latitude,
                lon=s.longitude,
                enabled=kalman_enabled,
                q=kf_q,
                r=kf_r,
            )
            s.latitude_kf = lat_kf
            s.longitude_kf = lon_kf

            add_raw = True
            if st.session_state.trail_raw:
                last_raw = st.session_state.trail_raw[-1]
                same_spot_raw = abs(last_raw["lat"] - s.latitude) < 1e-6 and abs(last_raw["lon"] - s.longitude) < 1e-6
                too_soon_raw = (now - last_raw["t"]) < 2.0
                if same_spot_raw and too_soon_raw:
                    add_raw = False
            if add_raw:
                st.session_state.trail_raw.append({"lat": s.latitude, "lon": s.longitude, "t": now})
                if len(st.session_state.trail_raw) > trail_limit:
                    st.session_state.trail_raw = st.session_state.trail_raw[-trail_limit:]

            add_kf = True
            if st.session_state.trail_kf:
                last_kf = st.session_state.trail_kf[-1]
                same_spot_kf = abs(last_kf["lat"] - lat_kf) < 1e-6 and abs(last_kf["lon"] - lon_kf) < 1e-6
                too_soon_kf = (now - last_kf["t"]) < 2.0
                if same_spot_kf and too_soon_kf:
                    add_kf = False
            if add_kf:
                st.session_state.trail_kf.append({"lat": lat_kf, "lon": lon_kf, "t": now})
                if len(st.session_state.trail_kf) > trail_limit:
                    st.session_state.trail_kf = st.session_state.trail_kf[-trail_limit:]


def start_reader(port: str, baud: int) -> None:
    if st.session_state.reader_thread and st.session_state.reader_thread.is_alive():
        return
    st.session_state.stop_evt.clear()
    t = threading.Thread(
        target=serial_worker,
        args=(port, int(baud), st.session_state.queue, st.session_state.stop_evt),
        daemon=True,
    )
    st.session_state.reader_thread = t
    t.start()


def stop_reader() -> None:
    st.session_state.stop_evt.set()
    st.session_state.state.connected = False


def render_status_banner(s: GPSState) -> None:
    if not s.connected:
        if (
            st.session_state.last_warning
            and (time.time() - st.session_state.last_warning_unix) < 8.0
            and TRANSIENT_READ_ERROR_TOKEN in st.session_state.last_warning.lower()
        ):
            st.warning("Transient USB serial drop detected. Auto-retrying connection...")
            return
        st.error("Serial disconnected. Check port/permissions, then press Connect.")
        return
    if s.fix is True:
        st.success("GPS lock acquired.")
        return
    if (s.sats_in_view or 0) > 0 and (s.satellites or 0) == 0:
        st.warning(f"Satellites in view: {s.sats_in_view}, but none used yet. Initial acquisition in progress.")
        return
    if s.satellites == 0:
        st.info("No satellites currently used. Move to clear sky and keep device steady.")
        return
    st.warning("Receiving satellites but no fix yet.")


def render_overview(s: GPSState, line_rate_10s: float, line_rate_30s: float) -> None:
    st.markdown("### Live Status")
    c1, c2, c3, c4, c5 = st.columns(5)
    c1.metric("Port", "Online" if s.connected else "Offline")
    c2.metric("Fix", "Yes" if s.fix else ("No" if s.fix is not None else "Unknown"))
    c3.metric("Sats Used", str(s.satellites) if s.satellites is not None else "-")
    c4.metric("Sats In View", str(s.sats_in_view) if s.sats_in_view is not None else "-")
    c5.metric("Quality", quality_band(s.hdop, s.snr_avg))

    q1, q2, q3, q4, q5 = st.columns(5)
    q1.metric("Fix Quality", str(s.fix_quality) if s.fix_quality is not None else "-")
    q2.metric("3D Mode", str(s.fix_3d) if s.fix_3d is not None else "-")
    q3.metric("Line Rate 10s", f"{line_rate_10s:.2f}/s")
    q4.metric("Line Rate 30s", f"{line_rate_30s:.2f}/s")
    q5.metric("Acq Score", f"{s.pre_fix_score}/100" if s.pre_fix_score is not None else "-")

def render_live_details(s: GPSState) -> None:
    st.markdown("### Live Details")

    pos_table = pd.DataFrame(
        [
            {"Field": "Latitude", "Value": f"{s.latitude:.6f}" if s.latitude is not None else "-"},
            {"Field": "Longitude", "Value": f"{s.longitude:.6f}" if s.longitude is not None else "-"},
            {"Field": "Altitude", "Value": f"{s.altitude_m:.2f} m" if s.altitude_m is not None else "-"},
            {"Field": "Speed", "Value": f"{s.speed_knots:.2f} kn" if s.speed_knots is not None else "-"},
        ]
    )
    st.dataframe(pos_table, hide_index=True, width="stretch")
    if s.latitude is not None and s.longitude is not None and s.latitude_kf is not None and s.longitude_kf is not None:
        sep_m = haversine_m(s.latitude, s.longitude, s.latitude_kf, s.longitude_kf)
        st.caption(
            f"Kalman: {s.latitude_kf:.6f}, {s.longitude_kf:.6f} | "
            f"Drift delta: {(s.latitude - s.latitude_kf):+.6f}, {(s.longitude - s.longitude_kf):+.6f} "
            f"({sep_m:.2f} m)"
        )

    dop_value = "-"
    if s.pdop is not None and s.hdop is not None and s.vdop is not None:
        dop_value = f"{s.pdop:.1f}/{s.hdop:.1f}/{s.vdop:.1f}"
    st.caption(f"DOP (P/H/V): {dop_value}")

    fresh_table = pd.DataFrame(
        [
            {"Event": "Any line", "Age": fmt_age(s.last_update_unix), "At": fmt_clock(s.last_update_unix)},
            {"Event": "Fix", "Age": fmt_age(s.last_fix_update_unix), "At": fmt_clock(s.last_fix_update_unix)},
            {"Event": "Location", "Age": fmt_age(s.last_loc_update_unix), "At": fmt_clock(s.last_loc_update_unix)},
            {"Event": "Total lines", "Age": str(s.total_lines), "At": "-"},
        ]
    )
    st.dataframe(fresh_table, hide_index=True, width="stretch")


def render_signal_panel(s: GPSState, history_window_min: int, chart_points: int) -> None:
    st.markdown("### RF / Satellites")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Sats Used", str(s.satellites) if s.satellites is not None else "-")
    c2.metric("Sats In View", str(s.sats_in_view) if s.sats_in_view is not None else "-")
    c3.metric("SNR Avg/Max", f"{s.snr_avg:.1f}/{s.snr_max}" if s.snr_avg is not None and s.snr_max is not None else "-")
    c4.metric("SNR Max Ever", str(s.snr_max_ever) if s.snr_max_ever is not None else "-")
    if s.pre_fix_score is not None:
        st.progress(int(max(0, min(100, s.pre_fix_score))))
        st.caption(f"Acquisition Stage: {s.pre_fix_stage or 'Unknown'}")

    if s.snr_bars is not None:
        if len(s.snr_bars) == 0:
            st.info("No SNR bars yet from GSV.")
        else:
            bars_df = pd.DataFrame(
                {
                    "sat": [f"SAT {i+1}" for i in range(len(s.snr_bars))],
                    "snr": s.snr_bars,
                }
            ).set_index("sat")
            st.caption("Signal bars (Betaflight-style)")
            st.bar_chart(bars_df["snr"], height=240)

    t1, t2 = st.columns(2)
    window_sec = float(history_window_min) * 60.0

    with t1:
        sat_rows = decimate_rows(recent_rows(st.session_state.sat_history, window_sec), chart_points)
        if sat_rows:
            sat_df = pd.DataFrame(sat_rows)
            sat_df["time"] = pd.to_datetime(sat_df["t"], unit="s")
            sat_df = sat_df.set_index("time")
            st.caption("Satellites Used Trend")
            st.line_chart(sat_df["sat"], height=150)
        view_rows = decimate_rows(recent_rows(st.session_state.view_history, window_sec), chart_points)
        if view_rows:
            view_df = pd.DataFrame(view_rows)
            view_df["time"] = pd.to_datetime(view_df["t"], unit="s")
            view_df = view_df.set_index("time")
            st.caption("Satellites In View Trend")
            st.line_chart(view_df["view"], height=150)
    with t2:
        snr_rows = decimate_rows(recent_rows(st.session_state.snr_history, window_sec), chart_points)
        if snr_rows:
            snr_df = pd.DataFrame(snr_rows)
            snr_df["time"] = pd.to_datetime(snr_df["t"], unit="s")
            snr_df = snr_df.set_index("time")
            st.caption("SNR Average Trend")
            st.line_chart(snr_df["snr_avg"], height=150)
        fix_rows = decimate_rows(recent_rows(st.session_state.fix_history, window_sec), chart_points)
        if fix_rows:
            fix_df = pd.DataFrame(fix_rows)
            fix_df["time"] = pd.to_datetime(fix_df["t"], unit="s")
            fix_df = fix_df.set_index("time")
            st.caption("Fix State Trend (0=no, 1=yes)")
            st.line_chart(fix_df["fix"], height=150)


def render_map(
    s: GPSState,
    center_map: bool,
    render_mode: str,
    style_choice: str,
    preview_lat: float,
    preview_lon: float,
    map_window_min: int,
    map_source: str,
) -> None:
    map_styles = {
        "Positron (Light)": "https://basemaps.cartocdn.com/gl/positron-gl-style/style.json",
        "Dark Matter": "https://basemaps.cartocdn.com/gl/dark-matter-gl-style/style.json",
        "Voyager": "https://basemaps.cartocdn.com/gl/voyager-gl-style/style.json",
        "No Basemap": None,
    }
    selected_map_style = map_styles[style_choice]

    st.markdown("### Map")
    raw_rows = recent_rows(st.session_state.trail_raw, float(map_window_min) * 60.0)
    kf_rows = recent_rows(st.session_state.trail_kf, float(map_window_min) * 60.0)
    source_rows = (
        kf_rows
        if map_source == "Kalman Filtered"
        else raw_rows if map_source == "Raw GPS" else (kf_rows if kf_rows else raw_rows)
    )
    if source_rows:
        df = pd.DataFrame(source_rows)
        latest = df.iloc[-1]

        if render_mode == "Simple" and map_source != "Both Overlay":
            if center_map:
                st.map(df.rename(columns={"lat": "latitude", "lon": "longitude"}), zoom=16)
            else:
                st.map(df.rename(columns={"lat": "latitude", "lon": "longitude"}))
        else:
            layers = []
            if map_source in ("Raw GPS", "Both Overlay") and raw_rows:
                raw_df = pd.DataFrame(raw_rows)
                layers.append(
                    pdk.Layer(
                        "PathLayer",
                        data=[{"path": raw_df[["lon", "lat"]].values.tolist()}],
                        get_path="path",
                        width_min_pixels=2,
                        get_color=[255, 70, 70, 180],
                    )
                )
                layers.append(
                    pdk.Layer(
                        "ScatterplotLayer",
                        data=raw_df,
                        get_position="[lon, lat]",
                        get_radius=7,
                        get_fill_color=[255, 70, 70, 200],
                        pickable=True,
                    )
                )
            if map_source in ("Kalman Filtered", "Both Overlay") and kf_rows:
                kf_df = pd.DataFrame(kf_rows)
                layers.append(
                    pdk.Layer(
                        "PathLayer",
                        data=[{"path": kf_df[["lon", "lat"]].values.tolist()}],
                        get_path="path",
                        width_min_pixels=3,
                        get_color=[40, 190, 90, 220],
                    )
                )
                layers.append(
                    pdk.Layer(
                        "ScatterplotLayer",
                        data=kf_df,
                        get_position="[lon, lat]",
                        get_radius=8,
                        get_fill_color=[40, 190, 90, 220],
                        pickable=True,
                    )
                )

            view = pdk.ViewState(
                latitude=float(latest["lat"] if center_map else df["lat"].mean()),
                longitude=float(latest["lon"] if center_map else df["lon"].mean()),
                zoom=15 if center_map else 13,
                pitch=30,
            )
            st.pydeck_chart(
                pdk.Deck(
                    map_style=selected_map_style,
                    initial_view_state=view,
                    layers=layers,
                    tooltip={"text": "Lat: {lat}\nLon: {lon}"},
                ),
                width="stretch",
            )

        legend = "Raw=Red | Kalman=Green" if map_source == "Both Overlay" else ""
        st.caption(
            f"Source: {map_source} | Track points shown: {len(df)} | "
            f"Latest: {latest['lat']:.6f}, {latest['lon']:.6f}"
            + (f" | {legend}" if legend else "")
        )
    else:
        preview_df = pd.DataFrame([{"latitude": float(preview_lat), "longitude": float(preview_lon)}])
        if render_mode == "Simple":
            st.map(preview_df, zoom=13)
        else:
            preview_layer = pdk.Layer(
                "ScatterplotLayer",
                data=[{"lat": float(preview_lat), "lon": float(preview_lon)}],
                get_position="[lon, lat]",
                get_radius=25,
                get_fill_color=[255, 180, 0, 220],
            )
            preview_view = pdk.ViewState(latitude=float(preview_lat), longitude=float(preview_lon), zoom=13, pitch=20)
            st.pydeck_chart(
                pdk.Deck(
                    map_style=selected_map_style,
                    initial_view_state=preview_view,
                    layers=[preview_layer],
                    tooltip={"text": "Preview center"},
                ),
                width="stretch",
            )
        st.info("Showing preview center until first location arrives.")


def render_diagnostics(s: GPSState, line_rate_30s: float) -> None:
    st.markdown("### Diagnostics")
    a1, a2, a3, a4, a5 = st.columns(5)
    a1.metric("Reconnects", st.session_state.connect_count)
    a2.metric("Errors", st.session_state.error_count)
    a3.metric("SNR Max Ever", str(s.snr_max_ever) if s.snr_max_ever is not None else "-")
    a4.metric("Raw Lines", len(st.session_state.raw_lines))
    a5.metric("Transient Drops", st.session_state.transient_count)

    p = st.session_state.parse_counts
    b1, b2, b3, b4, b5 = st.columns(5)
    b1.metric("fix", p["fix"])
    b2.metric("sat", p["sat"])
    b3.metric("view", p["view"])
    b4.metric("dop", p["dop"])
    b5.metric("snr", p["snr"])

    timing_table = pd.DataFrame(
        [
            {"Event": "Any line", "Last": fmt_clock(s.last_update_unix), "Age": fmt_age(s.last_update_unix)},
            {"Event": "Satellite line", "Last": fmt_clock(s.last_sat_update_unix), "Age": fmt_age(s.last_sat_update_unix)},
            {"Event": "Fix line", "Last": fmt_clock(s.last_fix_update_unix), "Age": fmt_age(s.last_fix_update_unix)},
            {"Event": "Location line", "Last": fmt_clock(s.last_loc_update_unix), "Age": fmt_age(s.last_loc_update_unix)},
        ]
    )
    st.dataframe(timing_table, hide_index=True, width="stretch")

    if line_rate_30s < 0.2:
        st.warning("Very low line rate. Check power, wiring, or baud.")
    if st.session_state.parse_counts["view"] == 0 and st.session_state.parse_counts["sat"] == 0:
        st.warning("No satellite metrics parsed yet.")
    if st.session_state.last_error and not s.connected:
        st.error(f"Last serial error: {st.session_state.last_error}")
    if st.session_state.last_warning and not s.connected:
        st.warning(f"Last transient warning: {st.session_state.last_warning}")


def render_raw() -> None:
    st.markdown("### Raw Serial")
    raw_order = st.radio("Order", ["Oldest -> Newest", "Newest -> Oldest"], horizontal=True)
    lines = st.session_state.raw_lines
    if raw_order == "Newest -> Oldest":
        lines = list(reversed(lines))
    st.text_area("Raw stream", value="\n".join(lines), height=420)


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    args, _ = parser.parse_known_args()

    st.set_page_config(page_title="GPS Live Dashboard", layout="wide")
    st.title("GPS Live Dashboard")
    st.caption("Live GPS quality + position dashboard. Disconnect/disable auto-connect before firmware upload.")

    port = st.sidebar.text_input("Serial Port", value=args.port)
    baud = st.sidebar.number_input("Baud", value=args.baud, step=1)
    refresh_ms = st.sidebar.slider("Refresh (ms)", min_value=200, max_value=3000, value=900, step=100)
    trail_limit = st.sidebar.slider("Trail Points", min_value=20, max_value=2000, value=400, step=20)
    raw_log_limit = st.sidebar.slider("Raw Log Lines", min_value=50, max_value=2000, value=300, step=50)
    chart_points = st.sidebar.slider("Chart Points", min_value=80, max_value=1200, value=320, step=20)
    history_window_min = st.sidebar.slider("Trend Window (min)", min_value=1, max_value=120, value=20, step=1)

    st.sidebar.markdown("### Map")
    center_map = st.sidebar.checkbox("Center on latest", value=True)
    preview_lat = st.sidebar.number_input("Preview Lat", value=38.897700, format="%.6f")
    preview_lon = st.sidebar.number_input("Preview Lon", value=-77.036500, format="%.6f")
    render_mode = st.sidebar.selectbox("Render", options=["Simple", "Styled"], index=1)
    map_window_min = st.sidebar.slider("Map Window (min)", min_value=1, max_value=180, value=45, step=1)
    map_source = st.sidebar.selectbox("Map Source", options=["Kalman Filtered", "Raw GPS", "Both Overlay"], index=0)
    style_choice = st.sidebar.selectbox(
        "Style",
        options=["Positron (Light)", "Dark Matter", "Voyager", "No Basemap"],
        index=0,
    )
    st.sidebar.markdown("### Kalman")
    kalman_enabled = st.sidebar.checkbox("Enable Kalman filter", value=st.session_state.get("kalman_enabled", True))
    kf_q = st.sidebar.number_input(
        "Process noise (Q)",
        min_value=1e-12,
        max_value=1e-4,
        value=float(st.session_state.get("kf_q", 1e-11)),
        format="%.12f",
    )
    kf_r = st.sidebar.number_input(
        "Measurement noise (R)",
        min_value=1e-10,
        max_value=1e-2,
        value=float(st.session_state.get("kf_r", 5e-7)),
        format="%.10f",
    )

    st.sidebar.markdown("### Runtime")
    auto_start = st.sidebar.checkbox("Auto-connect", value=True)
    pause_live_refresh = st.sidebar.checkbox("Pause live refresh", value=False)
    data_c1, data_c2 = st.sidebar.columns(2)
    clear_track = data_c1.button("Clear Track", width="stretch")
    clear_raw = data_c2.button("Clear Raw", width="stretch")
    data_c3, data_c4 = st.sidebar.columns(2)
    clear_trends = data_c3.button("Clear Trends", width="stretch")
    prune_now = data_c4.button("Prune Now", width="stretch")
    clear_all = st.sidebar.button("Clear All Data", width="stretch")

    init_session()

    if clear_track:
        st.session_state.trail_raw = []
        st.session_state.trail_kf = []
    if clear_raw:
        st.session_state.raw_lines = []
    if clear_trends:
        st.session_state.sat_history = []
        st.session_state.fix_history = []
        st.session_state.view_history = []
        st.session_state.snr_history = []
        st.session_state.line_times = []
    if prune_now:
        prune_runtime_data(raw_log_limit=raw_log_limit, trail_limit=trail_limit, history_limit=2000)
    if clear_all:
        clear_runtime_data(clear_state=False)
    st.session_state.kalman_enabled = kalman_enabled
    st.session_state.kf_q = float(kf_q)
    st.session_state.kf_r = float(kf_r)

    c1, c2, c3 = st.columns(3)
    if c1.button("Connect", width="stretch"):
        start_reader(port, int(baud))
    if c2.button("Disconnect", width="stretch"):
        stop_reader()
    if c3.button("Refresh now", width="stretch"):
        st.rerun()

    if auto_start and not st.session_state.connected_once:
        start_reader(port, int(baud))
        st.session_state.connected_once = True

    process_incoming(
        raw_log_limit=raw_log_limit,
        trail_limit=trail_limit,
        kalman_enabled=kalman_enabled,
        kf_q=float(kf_q),
        kf_r=float(kf_r),
    )
    s = st.session_state.state

    line_rate_10s = sum(1 for t in st.session_state.line_times if t >= time.time() - 10) / 10.0
    line_rate_30s = sum(1 for t in st.session_state.line_times if t >= time.time() - 30) / 30.0

    render_status_banner(s)
    st.caption(f"Last line: {s.last_line if s.last_line else '(none yet)'}")

    st.markdown("## Live View")
    render_overview(s, line_rate_10s, line_rate_30s)

    tab_overview, tab_signal, tab_diagnostics, tab_raw = st.tabs(
        ["Overview", "Signal", "Diagnostics", "Raw Serial"]
    )
    with tab_overview:
        left, right = st.columns([1.9, 1.1])
        with left:
            render_map(
                s,
                center_map,
                render_mode,
                style_choice,
                preview_lat,
                preview_lon,
                map_window_min,
                map_source,
            )
        with right:
            render_live_details(s)
    with tab_signal:
        render_signal_panel(s, history_window_min, chart_points)
    with tab_diagnostics:
        with st.container():
            render_diagnostics(s, line_rate_30s)
    with tab_raw:
        render_raw()

    effective_refresh_ms = refresh_ms
    if pause_live_refresh:
        st.info("Live refresh pause mode enabled. App runs in slow refresh until unpaused.")
        effective_refresh_ms = max(refresh_ms, 2500)

    time.sleep(effective_refresh_ms / 1000.0)
    st.rerun()


if __name__ == "__main__":
    main()
