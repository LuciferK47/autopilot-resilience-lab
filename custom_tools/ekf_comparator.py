import time
import os
import shutil
from pymavlink import mavutil
import matplotlib.pyplot as plt
from custom_tools.log_reader import get_log_data

# CONFIG
SITL_CONNECTION = 'udp:127.0.0.1:14550'
LOG_SOURCE_DIR = "../ardupilot/logs"
MY_LOG_DIR = "./logs"

def set_param(master, param_id, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_id.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)

def wait_for_arm(master, timeout=10.0):
    start = time.time()
    while time.time() - start < timeout:
        h = master.recv_match(type='HEARTBEAT', blocking=False)
        if h and (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
        time.sleep(0.1)
    return False

def run_test_flight(ekf_ver):
    print(f"\n=== STARTING FLIGHT FOR EKF{ekf_ver} ===")

    master = mavutil.mavlink_connection(SITL_CONNECTION)
    master.wait_heartbeat()

    set_param(master, "AHRS_EKF_TYPE", ekf_ver)
    set_param(master, "EKF2_ENABLE", 1 if ekf_ver == 2 else 0)
    set_param(master, "EKF3_ENABLE", 1 if ekf_ver == 3 else 0)

    print("Rebooting autopilot to apply EKF params...")
    master.reboot_autopilot()
    master.close()
    time.sleep(10)

    master = mavutil.mavlink_connection(SITL_CONNECTION)
    master.wait_heartbeat()

    print("Arming...")
    master.arducopter_arm()
    if not wait_for_arm(master, timeout=10):
        print("CRITICAL: Arming failed!")
        return None

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, 15)  # 15m

    print("Hovering for 20s (measuring drift)...")
    time.sleep(20)

    print("Landing...")
    master.set_mode("LAND")
    time.sleep(10)
    master.arducopter_disarm()

    # BUG FIX: use getmtime (not getctime) — getctime is inode-change time on Linux
    list_of_files = [os.path.join(LOG_SOURCE_DIR, f)
                     for f in os.listdir(LOG_SOURCE_DIR) if f.endswith('.BIN')]
    if not list_of_files:
        print("No .BIN logs found!")
        return None
    latest_file = max(list_of_files, key=os.path.getmtime)

    new_name = os.path.join(MY_LOG_DIR, f"flight_EKF{ekf_ver}.BIN")
    shutil.copy(latest_file, new_name)
    print(f"Log saved: {new_name}")
    return new_name

def compare_logs(log1, log2):
    if not log1 or not log2:
        print("Missing log files — cannot generate report.")
        return
    print("\n=== GENERATING REPORT ===")

    data1 = get_log_data(log1, 'NKF4', ['SV'])
    data2 = get_log_data(log2, 'NKF4', ['SV'])

    plt.figure(figsize=(10, 6))
    plt.plot(data1['SV'], label='EKF2 Velocity Variance', alpha=0.7)
    plt.plot(data2['SV'], label='EKF3 Velocity Variance', alpha=0.7, color='red')
    plt.title("EKF2 vs EKF3 Stability Comparison")
    plt.xlabel("Log Sample Index")
    plt.ylabel("Velocity Innovation Variance (SV)")
    plt.legend()
    plt.grid(True)
    plt.savefig("ekf_comparison_report.png", dpi=150)
    print("Report saved: ekf_comparison_report.png")

if __name__ == "__main__":
    os.makedirs(MY_LOG_DIR, exist_ok=True)
    l1 = run_test_flight(2)
    l2 = run_test_flight(3)
    compare_logs(l1, l2)