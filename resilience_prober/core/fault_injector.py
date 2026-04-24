"""
Fault Injector module for ArduPilot SITL resilience testing.
"""

import time
from pymavlink import mavutil

class FaultInjector:
    """
    Injects hardware and environmental faults into an ArduPilot SITL instance.
    """

    def __init__(self, master: mavutil.mavfile):
        """
        Initialize the FaultInjector with an active MAVLink connection.

        Args:
            master (mavutil.mavfile): Active MAVLink connection.
        """
        self.master = master

    def set_sim_parameter(self, param_id: str, param_value: float, timeout: float = 5.0) -> None:
        """
        Set a SIM_ parameter and actively wait for confirmation.
        
        Args:
            param_id (str): The string ID of the parameter.
            param_value (float): The value to set.
            timeout (float): Total time to wait for confirmation.
        
        Raises:
            RuntimeError: If the parameter change is not confirmed within the timeout.
        """
        start_time = time.time()
        last_send_time = 0.0
        
        while time.time() - start_time < timeout:
            current_time = time.time()
            if current_time - last_send_time >= 1.0:
                # Send parameter set command
                self.master.param_set_send(
                    param_id,
                    param_value,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )
                last_send_time = current_time
            
            # Check for PARAM_VALUE messages
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
            if msg:
                received_param_id = msg.param_id
                if isinstance(received_param_id, bytes):
                    try:
                        received_param_id = received_param_id.decode('utf-8')
                    except UnicodeDecodeError:
                        pass
                
                if isinstance(received_param_id, str):
                    # ArduPilot null-terminates string arrays, strip trailing nulls
                    received_param_id = received_param_id.rstrip('\x00')
                
                if received_param_id == param_id:
                    # Check if the value effectively matches
                    if abs(msg.param_value - param_value) < 0.0001:
                        # Success
                        print(f"[FaultInjector] Confirmed {param_id} = {param_value}")
                        return
                    else:
                        print(f"[FaultInjector] Warning: {param_id} changed, but value is {msg.param_value} (expected {param_value})")
        
        raise RuntimeError(f"Failed to confirm parameter {param_id} change to {param_value} within {timeout} seconds.")

    def wait_for_kinematic_trigger(self, target_alt: float = 15.0, target_gs: float = 5.0, target_wp_dist: float = 15.0, timeout: float = 300.0) -> bool:
        """
        Blocks until the vehicle breaches the exact kinematic flight envelope.
        Returns True once the envelope is breached.
        """
        print(f"Waiting for Kinematic Trigger (Alt > {target_alt}m, GS > {target_gs}m/s, WP Dist < {target_wp_dist}m)...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # We poll VFR_HUD (Alt/GS) and NAV_CONTROLLER_OUTPUT (Waypoint Distance)
            msg = self.master.recv_match(type=['VFR_HUD', 'NAV_CONTROLLER_OUTPUT'], blocking=True, timeout=0.5)
            
            if msg is not None:
                if msg.get_type() == 'VFR_HUD':
                    self._current_alt = msg.alt
                    self._current_gs = msg.groundspeed
                elif msg.get_type() == 'NAV_CONTROLLER_OUTPUT':
                    self._current_wp_dist = msg.wp_dist

                # Check state machine if all variables are populated
                if hasattr(self, '_current_alt') and hasattr(self, '_current_wp_dist') and hasattr(self, '_current_gs'):
                    if self._current_alt > target_alt and self._current_gs > target_gs and self._current_wp_dist < target_wp_dist:
                        print(">>> Kinematic Envelope Breached! Triggering Fault! <<<")
                        return True
                        
        raise TimeoutError("Failed to reach kinematic trigger envelope within timeout.")

    def inject_gps_denial(self) -> int:
        """
        Simulates total GPS loss by disabling the simulated GPS.
        Returns the exact TIME_BOOT timeus epoch.
        """
        print("[FaultInjector] Acquiring SYSTEM_TIME for epoch synchronization...")
        msg = self.master.recv_match(type='SYSTEM_TIME', blocking=True, timeout=2.0)
        if not msg:
            raise RuntimeError("Failed to receive SYSTEM_TIME msg.")
        
        fault_time_us = int(msg.time_boot_ms) * 1000
        print(f"[FaultInjector] Synced Fault TimeUS: {fault_time_us}")

        print("[FaultInjector] Injecting GPS Denial...")
        try:
            self.set_sim_parameter("SIM_GPS_DISABLE", 1.0, timeout=1.0)
        except RuntimeError:
            print("[FaultInjector] SIM_GPS_DISABLE parameter not found or failed, falling back to SIM_GPS1_ENABLE=0.0")
            self.set_sim_parameter("SIM_GPS1_ENABLE", 0.0, timeout=2.0)
            
        return fault_time_us

    def restore_gps(self) -> None:
        """
        Restores the simulated GPS.
        """
        print("[FaultInjector] Restoring GPS...")
        try:
            self.set_sim_parameter("SIM_GPS_DISABLE", 0.0, timeout=1.0)
        except RuntimeError:
            self.set_sim_parameter("SIM_GPS1_ENABLE", 1.0, timeout=2.0)
