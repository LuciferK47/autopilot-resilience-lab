import time
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from pymavlink import mavutil
from log_reader import get_log_data

SITL_CONNECTION = 'udp:127.0.0.1:14550'
LOG_SOURCE_DIR = "../ardupilot/logs"
MY_LOG_DIR = "./logs"

# ArduCopter mode numbers (correct mapping)
MODES = {
    "STABILIZE": 0,
    "ALT_HOLD":  2,
    "LOITER":    5,
    "LAND":      9,
    "POSHOLD":   16,
    "SYSTEMID":  25,   
}

def set_param(master, param_id, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_id.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.1)

def get_param(master, param_id, timeout=2.0):
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param_id.encode('utf-8'), -1
    )
    start = time.time()
    while time.time() - start < timeout:
        m = master.recv_match(type='PARAM_VALUE', blocking=False)
        if m and m.param_id.strip('\x00') == param_id:
            return m.param_value
        time.sleep(0.05)
    return None

def wait_for_mode(master, mode_num, timeout=5.0):
    """Block until the heartbeat confirms the target mode, or timeout."""
    start = time.time()
    while time.time() - start < timeout:
        h = master.recv_match(type='HEARTBEAT', blocking=False)
        if h and h.custom_mode == mode_num:
            return True
        # Print any rejection messages
        s = master.recv_match(type='STATUSTEXT', blocking=False)
        if s:
            print(f"  FC: {s.text}")
        time.sleep(0.1)
    return False

def set_mode_confirmed(master, mode_num, mode_name="", timeout=5.0):
    """Set mode and wait for confirmation. Returns True on success."""
    master.set_mode(mode_num)
    success = wait_for_mode(master, mode_num, timeout)
    label = mode_name or str(mode_num)
    if success:
        print(f"  ✓ Mode confirmed: {label}")
    else:
        print(f"  ✗ Mode change FAILED for {label}")
    return success

def wait_for_arm(master, timeout=10.0):
    """Wait until the vehicle reports armed in heartbeat."""
    start = time.time()
    while time.time() - start < timeout:
        h = master.recv_match(type='HEARTBEAT', blocking=False)
        if h and (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        time.sleep(0.1)
    return False

def wait_for_altitude(master, target_alt_m, tolerance=1.5, timeout=30):
    """Poll VFR_HUD until the drone reaches target altitude."""
    print(f"  Waiting for altitude ≥ {target_alt_m - tolerance:.1f} m ...")
    start = time.time()
    while time.time() - start < timeout:
        m = master.recv_match(type='VFR_HUD', blocking=False)
        if m and m.alt >= (target_alt_m - tolerance):
            print(f"  ✓ Altitude reached: {m.alt:.1f} m")
            return True
        time.sleep(0.2)
    print(f"  ✗ Altitude timeout (last known alt may be low)")
    return False

def reconnect(connection_str, wait=15):
    """Close, sleep, and reconnect."""
    print(f"Reconnecting in {wait}s ...")
    time.sleep(wait)
    master = mavutil.mavlink_connection(connection_str)
    master.wait_heartbeat()
    print("  ✓ Reconnected.")
    return master

def save_latest_log(source_dir, dest_path):
    """Find newest .BIN in source_dir and copy to dest_path."""
    bins = [os.path.join(source_dir, f)
            for f in os.listdir(source_dir) if f.endswith('.BIN')]
    if not bins:
        print("  ✗ No .BIN logs found!")
        return None
    # Use getmtime (not getctime) — reliable on Linux
    latest = max(bins, key=os.path.getmtime)
    shutil.copy(latest, dest_path)
    print(f"  Log saved: {dest_path}")
    return dest_path

# ──────────────────────────────────────────────────────────────────────────────

def run_sysid_sweep():
    print("\n=== STARTING SYSID PROTOCOL (DIAGNOSTIC) ===")
    master = mavutil.mavlink_connection(SITL_CONNECTION)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system} / component {master.target_component}")

    # ── STAGE 1: PARAMETER SETUP ──────────────────────────────────────────────
    print("\n[STAGE 1] Checking SID_AXIS...")
    current_axis = get_param(master, "SID_AXIS")
    print(f"  SID_AXIS = {current_axis}")

    if current_axis != 1.0:
        print("  Setting SID_AXIS=1 and rebooting...")
        set_param(master, "SID_AXIS", 1)
        set_param(master, "LOG_BITMASK", 131071)
        set_param(master, "LOG_DISARMED", 1)
        time.sleep(1)
        master.reboot_autopilot()
        master.close()

        master = reconnect(SITL_CONNECTION, wait=15)

        new_axis = get_param(master, "SID_AXIS")
        if new_axis != 1.0:
            print(f"  CRITICAL: SID_AXIS still {new_axis}!")
            print("  SOLUTION: Restart sim_vehicle.py WITHOUT the '-w' (wipe) flag.")
            return None
        print("  Parameter persisted successfully.")
    else:
        print("  SID_AXIS is already 1. System ready.")

    # ── STAGE 2: SYSID CONFIGURATION ─────────────────────────────────────────
    print("\n[STAGE 2] Configuring SysID sweep parameters...")
    sysid_params = {
        "SID_F_START":  2.0,    # Hz start
        "SID_F_STOP":  15.0,    # Hz stop
        "SID_T_REC":   15.0,    # seconds of recording
        "SID_T_FADE_IN": 1.0,   # fade-in ramp
    }
    for p, v in sysid_params.items():
        set_param(master, p, v)
        print(f"  {p} = {v}")

    # ── STAGE 3: FLIGHT ───────────────────────────────────────────────────────
    print("\n[STAGE 3] Flight preparation...")

    # Set LOITER *before* arming so FC accepts it
    if not set_mode_confirmed(master, MODES["LOITER"], "LOITER"):
        print("  WARNING: Could not confirm LOITER — trying ALT_HOLD as fallback.")
        set_mode_confirmed(master, MODES["ALT_HOLD"], "ALT_HOLD")

    print("  Arming...")
    master.arducopter_arm()
    if not wait_for_arm(master, timeout=10):
        print("  CRITICAL: Arming failed. Check pre-arm messages.")
        # Print any STATUSTEXT
        for _ in range(10):
            s = master.recv_match(type='STATUSTEXT', blocking=False)
            if s: print(f"    FC: {s.text}")
            time.sleep(0.1)
        return None
    print("  ✓ Armed.")

    TARGET_ALT = 15  # metres
    print(f"  Sending TAKEOFF to {TARGET_ALT} m ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,          # confirmation
        0, 0, 0, 0, # params 1-4 unused
        0, 0,       # lat, lon unused for copter
        TARGET_ALT  # altitude
    )

    if not wait_for_altitude(master, TARGET_ALT, timeout=30):
        print("  WARNING: Did not reach target altitude in time — proceeding anyway.")

    # ── STAGE 4: SYSID ENGAGE ─────────────────────────────────────────────────
    print("\n[STAGE 4] Engaging SYSID mode (mode 25)...")
    if not set_mode_confirmed(master, MODES["SYSTEMID"], "SYSTEMID", timeout=5):
        print("  FAILURE: SYSID mode rejected.")
        print("  Possible causes:")
        print("    • ArduCopter not compiled with SYSID support (check Features during build)")
        print("    • SID_AXIS not persisted — run without '-w' flag in sim_vehicle.py")
        print("    • Altitude too low — arm and climb higher before switching")
        master.set_mode(MODES["LAND"])
        return None

    print("  ✓ SYSID mode engaged! Running frequency sweep...")
    # Monitor for the full sweep + a small buffer
    sweep_duration = sysid_params["SID_T_REC"] + sysid_params["SID_T_FADE_IN"] + 3
    for elapsed in range(int(sweep_duration)):
        s = master.recv_match(type='STATUSTEXT', blocking=False)
        if s: print(f"  FC: {s.text}")
        time.sleep(1)
        print(f"  Sweeping... {elapsed + 1}/{int(sweep_duration)}s", end='\r')
    print()

    # ── STAGE 5: LAND + SAVE ──────────────────────────────────────────────────
    print("\n[STAGE 5] Landing...")
    set_mode_confirmed(master, MODES["LAND"], "LAND")
    time.sleep(12)  # Wait for touchdown
    master.arducopter_disarm()
    print("  Disarmed.")

    time.sleep(3)   # Let FC flush log to disk
    os.makedirs(MY_LOG_DIR, exist_ok=True)
    dest = os.path.join(MY_LOG_DIR, "sysid_sweep.BIN")
    return save_latest_log(LOG_SOURCE_DIR, dest)

# ──────────────────────────────────────────────────────────────────────────────

def generate_bode(log_file):
    if not log_file:
        print("No log file provided — skipping Bode plot.")
        return
    print(f"\n=== GENERATING BODE PLOT from {log_file} ===")
    data = get_log_data(log_file, 'SIDS', ['Targ', 'Gyr'])

    if not data['Targ']:
        print("ERROR: No SIDS messages found in log.")
        print("  Possible fix: Confirm SYSID mode actually ran during flight.")
        return

    FS = 400  # Hz — ArduPilot SIDS log rate
    u = np.array(data['Targ']) - np.mean(data['Targ'])
    y = np.array(data['Gyr'])  - np.mean(data['Gyr'])

    # Cross-power spectral density → frequency response
    f, Pxy = signal.csd(u, y, FS, nperseg=1024)
    f, Pxx = signal.welch(u, FS, nperseg=1024)
    H = Pxy / (Pxx + 1e-10)

    mag_db   = 20 * np.log10(np.abs(H) + 1e-10)
    phase_deg = np.degrees(np.angle(H))
    coherence  = np.abs(Pxy)**2 / (Pxx * (np.abs(signal.welch(y, FS, nperseg=1024)[1])) + 1e-10)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("System ID: Roll Axis — Bode Plot", fontsize=14)

    axes[0].semilogx(f, mag_db)
    axes[0].set_ylabel("Magnitude (dB)")
    axes[0].grid(True, which="both")

    axes[1].semilogx(f, phase_deg)
    axes[1].set_ylabel("Phase (°)")
    axes[1].grid(True, which="both")

    axes[2].semilogx(f, coherence)
    axes[2].set_ylabel("Coherence")
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylim([0, 1.1])
    axes[2].axhline(0.8, color='r', linestyle='--', label='0.8 threshold')
    axes[2].legend()
    axes[2].grid(True, which="both")

    out = "sysid_bode_plot.png"
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"  ✓ Bode plot saved: {out}")

# ──────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    os.makedirs(MY_LOG_DIR, exist_ok=True)
    log = run_sysid_sweep()
    generate_bode(log)