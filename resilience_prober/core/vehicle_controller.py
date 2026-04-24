import time
from typing import Optional, Iterator

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class ArmingTimeoutError(Exception):
    """Raised when the vehicle fails to arm within the specified timeout."""
    pass

class VehicleController:
    """
    Handles robust MAVLink communication for connecting, arming, and taking off
    a simulated ArduPilot vehicle. Strictly separated from plotting and log analysis.
    """

    def __init__(self) -> None:
        """Initializes the VehicleController with no active connection."""
        self.master: Optional[mavutil.mavfile] = None

    def apply_environment_params(self, env_params: dict) -> None:
        """Applies baseline environmental noise and wind parameters to SITL."""
        if not env_params:
            return
            
        print("Injecting Environmental Determinism Parameters...")
        for param_id, param_value in env_params.items():
            # MAVLink requires the param_id to be exactly 16 bytes padded
            param_id_bytes = param_id.encode('utf-8')[:16].ljust(16, b'\x00')
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_id_bytes,
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            print(f"  -> Set {param_id} = {param_value}")
        time.sleep(1.0) # Give SITL a moment to digest parameter changes

    def connect(self, connection_string: str) -> None:
        try:
            print(f"Connecting to vehicle on: {connection_string}")
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat(timeout=30.0)
            if not self.master.target_system:
                raise ConnectionError("Timeout waiting for initial MAVLink heartbeat.")
            print(f"Heartbeat received from system {self.master.target_system}")
            
            # Request all data streams (vital when connecting directly without MAVProxy)
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavlink2.MAV_DATA_STREAM_ALL,
                10, # Hz
                1 # start
            )
            print("Requested data streams at 10Hz")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to the vehicle: {e}") from e

    def wait_for_gps_lock(self, timeout: float = 120.0) -> None:
        """Waits for the vehicle to acquire a 3D GPS lock."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        print("Waiting for GPS 3D fix...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1.0)
            if msg is not None:
                if msg.fix_type >= 3:
                    print(">>> 3D GPS Lock Acquired! <<<")
                    return

        raise TimeoutError(f"Vehicle failed to acquire 3D GPS lock within {timeout} seconds.")

    def wait_for_ekf_ready(self, timeout: float = 120.0) -> None:
        """Blocks until the EKF mathematically reports stable absolute position."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        print("Waiting for EKF to mathematically stabilize (EKF_STATUS_REPORT)...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
            if msg is not None:
                # Check for EKF_POS_HORIZ_ABS (16) and EKF_PRED_POS_HORIZ_ABS (512)
                if (msg.flags & 16) and (msg.flags & 512):
                    print(">>> EKF Position Estimate is Stable and Trusted! <<<")
                    # Add a brief 2-second buffer to allow the FCU's PreArm cache to clear
                    time.sleep(2.0)
                    return

        raise TimeoutError("EKF failed to stabilize within the timeout period.")

    def set_mode(self, target_mode: str, timeout: float = 10.0) -> None:
        """Robustly switches flight mode and blocks until FCU confirms."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")
        
        mode_id = self.master.mode_mapping().get(target_mode)
        if mode_id is None:
            raise ValueError(f"Unknown mode: {target_mode}")

        print(f"Requesting mode switch to: {target_mode}")
        start_time = time.time()
        last_req_time = 0.0

        while time.time() - start_time < timeout:
            # Send/Re-send the mode switch request every 2 seconds
            if time.time() - last_req_time > 2.0:
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id
                )
                last_req_time = time.time()

            # Listen for Heartbeats AND StatusText (to catch errors)
            msg = self.master.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1.0)
            if msg is not None:
                if msg.get_type() == 'STATUSTEXT':
                    print(f"FCU Status: {msg.text}")
                elif msg.get_type() == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                    if msg.custom_mode == mode_id:
                        print(f">>> Mode confirmed: {target_mode} <<<")
                        return

        raise TimeoutError(f"FCU refused to switch to {target_mode} within {timeout} seconds.")
        
    def arm_vehicle(self, timeout: float = 60.0) -> None:
        """
        Attempts to arm the vehicle. If rejected due to Pre-Arm checks, it captures
        the STATUSTEXT reasons and retries until the timeout expires.
        """
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        self.set_mode("GUIDED", timeout=timeout)
        time.sleep(1) # Give it a second to process the mode switch

        print("Initiating arming sequence. Waiting for EKF to pass Pre-Arm checks...")
        start_time = time.time()
        last_arm_request = 0.0

        while time.time() - start_time < timeout:
            # Re-send the arm command every 5 seconds if we aren't armed yet
            if time.time() - last_arm_request > 5.0:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                last_arm_request = time.time()

            # Listen for Heartbeats (to verify success) AND StatusText (to catch errors)
            msg = self.master.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1.0)
            
            if msg is not None:
                if msg.get_type() == 'HEARTBEAT':
                    if msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("\n>>> Vehicle is verified ARMED and ready for takeoff! <<<")
                        return
                
                elif msg.get_type() == 'STATUSTEXT':
                    text = msg.text
                    # We only care about PreArm failures right now
                    if "PreArm" in text:
                        print(f"FCU Rejection: {text}")

        raise ArmingTimeoutError(f"Vehicle did not pass Pre-Arm checks within {timeout} seconds.")

    def takeoff(self, target_altitude: float, timeout: float = 60.0) -> None:
        """Commands the vehicle to takeoff, ensuring mode synchronization first."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        if target_altitude <= 0:
            raise ValueError("Target altitude must be strictly positive.")

        # 1. Synchronize mode safely using the new robust method
        self.set_mode("GUIDED")

        # 2. Command takeoff
        print(f"Commanding takeoff to {target_altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavlink2.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude
        )

        # 3. Block until altitude is reached
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg is not None:
                current_alt_m = msg.relative_alt / 1000.0
                if current_alt_m >= target_altitude * 0.95:
                    print(f"Target altitude reached. Current altitude: {current_alt_m:.2f}m")
                    return

        raise TimeoutError(f"Vehicle failed to reach {target_altitude}m within {timeout} seconds.")

    def upload_mission(self, waypoints: list[tuple[float, float, float]], timeout: float = 10.0) -> None:
        """
        Uploads a list of (Latitude, Longitude, Altitude) waypoints using the strict MAVLink mission protocol.
        """
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        print("Clearing existing mission...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
            mavlink2.MAV_MISSION_TYPE_MISSION
        )

        ack_clear_time = time.time()
        clear_success = False
        while time.time() - ack_clear_time < timeout:
            msg = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=1.0)
            if msg is not None:
                if msg.type == mavlink2.MAV_MISSION_ACCEPTED:
                    clear_success = True
                    break
        
        if not clear_success:
            raise TimeoutError("Timeout waiting for MISSION_ACK after clearing mission.")

        count = len(waypoints)
        if count == 0:
            return

        print(f"Uploading {count} waypoints...")
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            count,
            mavlink2.MAV_MISSION_TYPE_MISSION
        )

        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=0.5)
            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == 'MISSION_ACK':
                    if msg.type == mavlink2.MAV_MISSION_ACCEPTED:
                        print("Mission upload successfully acknowledged.")
                        return
                    else:
                        raise RuntimeError(f"Mission upload rejected with type {msg.type}")

                elif msg_type in ['MISSION_REQUEST_INT', 'MISSION_REQUEST']:
                    seq = msg.seq
                    if seq >= count:
                        raise IndexError(f"Vehicle requested invalid waypoint sequence {seq}")

                    lat, lon, alt = waypoints[seq]
                    lat_int = int(lat * 1e7)
                    lon_int = int(lon * 1e7)

                    # First waypoint should ideally be a TAKEOFF command for AUTO mode
                    cmd = mavlink2.MAV_CMD_NAV_TAKEOFF if seq == 0 else mavlink2.MAV_CMD_NAV_WAYPOINT
                    self.master.mav.mission_item_int_send(
                        self.master.target_system,
                        self.master.target_component,
                        seq,
                        mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        cmd,
                        0,  # current
                        1,  # autocontinue
                        0, 0, 0, 0,  # params 1-4
                        lat_int, lon_int, alt,
                        mavlink2.MAV_MISSION_TYPE_MISSION
                    )

        raise TimeoutError("Timeout waiting for mission upload handshake to complete.")

    def start_mission(self) -> None:
        """Changes the vehicle mode to AUTO to begin executing the uploaded mission."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        # Provide mid-stick RC to prevent auto-disarm when switching modes with 0 throttle.
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            1500, 1500, 1500, 1500, 0, 0, 0, 0
        )
        time.sleep(0.5)

        self.set_mode("AUTO")

    def monitor_mission_progress(self) -> Iterator[int]:
        """A generator method that yields the current waypoint index."""
        if self.master is None:
            raise RuntimeError("Vehicle is not connected.")

        while True:
            msg = self.master.recv_match(type=['MISSION_CURRENT', 'MISSION_ITEM_REACHED'], blocking=True, timeout=0.5)
            if msg is not None:
                yield msg.seq

# --- Execution Block for Testing ---
if __name__ == "__main__":
    controller = VehicleController()
    
    try:
        # Using safely mapped UDP to avoid Address already in use MAVProxy TCP errors
        controller.connect("udp:127.0.0.1:14550")
        
        # The arm_vehicle method will now intelligently wait and retry for up to 60 seconds.
        controller.arm_vehicle(timeout=60.0)
        
        controller.takeoff(target_altitude=10.0)
        
        print("Test complete. Holding altitude for 5 seconds before exiting.")
        time.sleep(5)
        
    except Exception as e:
        print(f"\nTEST FAILED: {e}")
