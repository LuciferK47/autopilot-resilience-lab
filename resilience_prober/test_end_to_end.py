import os
import sys
import time
import subprocess
import pandas as pd
from pathlib import Path
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from core.config_loader import ConfigLoader
from core.vehicle_controller import VehicleController
from core.fault_injector import FaultInjector
from core.log_analyzer import EKFForensics
from core.resilience_calculator import ResilienceCalculator
from core.report_generator import export_artifacts, print_delta_scorecard

class LogManager:
    @staticmethod
    def extract_latest_log(log_dir: str) -> Path:
        """Finds the most freshly modified .BIN log within the target directory."""
        log_path = Path(log_dir)
        bin_files = list(log_path.glob("*.BIN"))
        if not bin_files:
            raise FileNotFoundError(f"No .BIN files found in directory: {log_dir}")
        
        latest_log = max(bin_files, key=lambda f: f.stat().st_mtime)
        return latest_log

def run_scenario(scenario_name: str, loader: ConfigLoader, ardupilot_dir: str) -> tuple[pd.DataFrame, dict]:
    print(f"\n=== Executing Scenario: {scenario_name} ===")
    scenario = loader.load_scenario(scenario_name)
    
    print("Booting SITL Instance (Wiping EEPROM)...")
    sitl_proc = subprocess.Popen(
        ['Tools/autotest/sim_vehicle.py', '-v', 'ArduCopter', '-f', 'quad', '--no-rebuild', '--no-mavproxy', '-w'], 
        stdout=subprocess.DEVNULL, 
        stderr=subprocess.DEVNULL, 
        cwd=ardupilot_dir
    )
    time.sleep(20)

    fault_time_us = None

    try:
        controller = VehicleController()
        print("Connecting to TCP 127.0.0.1:5760...")
        controller.connect("tcp:127.0.0.1:5760")
        
        # Step 1: Environmental Determinism
        env_params = scenario.get("environment", {})
        controller.apply_environment_params(env_params)

        mission_data = scenario.get("mission", [])
        mission_waypoints = [(wp['lat'], wp['lon'], wp['alt']) for wp in mission_data]
        
        controller.wait_for_gps_lock()
        controller.wait_for_ekf_ready()
        
        controller.upload_mission(mission_waypoints)
        controller.set_mode("GUIDED", timeout=30.0)
        controller.arm_vehicle()

        print("Lifting off before starting mission...")
        controller.takeoff(15.0)

        controller.start_mission()

        injector = FaultInjector(controller.master)
        fault_triggers = scenario.get("fault_triggers", [])
        
        if fault_triggers and fault_triggers[0].get("trigger") == "kinematic":
            # Step 2: Spatial Fault Trigger
            if injector.wait_for_kinematic_trigger(target_wp_dist=15.0):
                fault_time_us = injector.inject_gps_denial()
        else:
            # Baseline Epoch Sync for Nominal
            print("[Nominal] Acquiring SYSTEM_TIME for baseline epoch synchronization...")
            msg = controller.master.recv_match(type='SYSTEM_TIME', blocking=True, timeout=2.0)
            if msg:
                fault_time_us = int(msg.time_boot_ms) * 1000

        print("Monitoring Waypoint Progress...")
        start_time = time.time()
        max_duration = 300.0 if fault_triggers else 60.0 # Shorter duration for nominal
        
        while time.time() - start_time < max_duration:
            msg_hb = controller.master.recv_match(type="HEARTBEAT", blocking=False)
            if msg_hb:
                armed = (msg_hb.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if not armed:
                    print("Vehicle Disarmed (Crash/Failsafe triggered). Flight Over.")
                    break
            time.sleep(0.1)

    except Exception as e:
        print(f"Execution Error: {e}")
        
    finally:
        if 'sitl_proc' in locals():
            # --- THE UNIVERSAL I/O COOLDOWN ---
            print("Initiating 5-second IO Cooldown to flush DataFlash buffer...")
            time.sleep(5.0)
            
            print("Terminating SITL subprocess...")
            sitl_proc.kill() # Aggressively kill the wrapper
            
            # --- THE ZOMBIE SLAYER ---
            import os
            print("Purging zombie ArduCopter processes to free TCP 5760...")
            os.system("killall -9 arducopter sim_vehicle.py mavproxy.py 2>/dev/null")
            time.sleep(3.0) # Give the Linux kernel time to release the port
            # -------------------------

    if fault_time_us is None:
        raise ValueError("Fault/Epoch time was never generated.")

    print("Locating logs...")
    log_dir = os.path.join(ardupilot_dir, "logs")
    latest_bin = LogManager.extract_latest_log(log_dir)
    print(f"Processing Logs from {latest_bin}...")
    
    forensics = EKFForensics(latest_bin)
    df = forensics.extract_ekf_data(fault_time_us=fault_time_us)

    print("Calculating Matrix Resilience Metrics...")
    calculator = ResilienceCalculator(df, fault_time_us=fault_time_us)
    metrics = {
        'peak_error': calculator.calculate_peak_error(),
        'saturation_pct': calculator.calculate_control_saturation(),
        'resilience_index': calculator.compute_resilience_index()
    }
    return df, metrics

def main() -> None:
    config_path = os.path.join(os.path.dirname(__file__), "config", "scenarios.yaml")
    loader = ConfigLoader(config_path)
    ardupilot_dir = "/home/kushagrasingh/autopilot-resilience-lab/ardupilot"

    print("=== ArduPilot Resilience Testing Architecture ===")
    
    # Step 3: Baseline Delta Architecture (Nominal First)
    nominal_df, nominal_metrics = run_scenario("nominal", loader, ardupilot_dir)
    
    # Step 4: Artifact Duality
    export_artifacts("nominal", nominal_df, nominal_metrics)
    
    # Run Fault Scenario
    fault_df, fault_metrics = run_scenario("gps_denial_basic", loader, ardupilot_dir)
    
    print_delta_scorecard("gps_denial_basic", nominal_metrics, fault_metrics)
    export_artifacts("gps_denial_basic", fault_df, fault_metrics)

if __name__ == "__main__":
    main()
