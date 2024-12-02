import time
from pymavlink import mavutil

def drone_connect(udp_port):
    print(f"Connecting to drone on UDP port {udp_port}...")
    drone = mavutil.mavlink_connection(f'udp:localhost:{udp_port}')

    drone.wait_heartbeat()
    print(f"Drone on port {udp_port} connected!")

    return drone

def drone_reboot(drone, target_system):
    print(f"[System ID: {target_system}] Rebooting Drone...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print(f"[System ID: {target_system}] Rebooting Completed!")

def drone_set_mode(drone, target_system):
    for _ in range(10):
        drone.mav.set_position_target_local_ned_send(
            0,
            target_system,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), # position, velocity, yaw control mask
            0, 0, -10, # x, y, z position
            0, 0, 0, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration/force
            0, 0 # yaw, yaw rate
        )
        time.sleep(0.1)

    print(f"[System ID: {target_system}] Setting drone mode...")
    PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0
    )
    print(f"[System ID: {target_system}] Setting Completed!")
        
def drone_arm(drone, target_system):
    print(f"[System ID: {target_system}] Arming drone...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    drone.motors_armed_wait()
    print(f"[System ID: {target_system}] Drone Armed!")
    
    time.sleep(5)

def drone_takeoff(drone, target_system, target_altitude):
    print(f"[System ID: {target_system}] Taking off to altitude {abs(target_altitude)} meters...")
    """
    current_altitude = 0
    while current_altitude > target_altitude:
        current_altitude -= step
        drone.mav.set_position_target_local_ned_send(
            0,
            target_system,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), # position, velocity, yaw control mask
            0, 0, current_altitude, # x, y, z position
            0, 0, 0, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration/force
            0, 0 # yaw, yaw rate
        )
        time.sleep(0.1)
    """
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_altitude
    )
    print(f"[System ID: {target_system}] Takeoff!")

    time.sleep(5)

def drone_hold(drone, target_system, duration=5, target_altitude=-20):
    for _ in range(int(duration) * 10):
        drone.mav.set_position_target_local_ned_send(
            0,
            target_system,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), # position, velocity, yaw control mask
            0, 0, target_altitude, # x, y, z position
            0, 0, 0, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration/force
            0, 0 # yaw, yaw rate
        )

def drone_land(drone, target_system, step=0.5):
    print(f"[System ID: {target_system}] Landing...")
    """
    current_altitude = -20
    while current_altitude < 0:
        current_altitude += step
        drone.mav.set_position_target_local_ned_send(
            0,
            target_system,
            0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), # position, velocity, yaw control mask
            0, 0, current_altitude, # x, y, z position
            0, 0, 0, # x, y, z velocity
            0, 0, 0, # x, y, z acceleration/force
            0, 0 # yaw, yaw rate
        )
        time.sleep(0.1)
    """
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print(f"[System ID: {target_system}] Land!")

def main():
    leader = drone_connect(14540)
    follower = drone_connect(14541)

    print("\n")

    drone_reboot(leader, target_system=1)
    drone_reboot(follower, target_system=2)

    print("\n")

    drone_arm(leader, target_system=1)
    drone_arm(follower, target_system=2)

    print("\n")

    # drone_set_mode(leader, target_system=1)
    # drone_set_mode(follower, target_system=2)

    print("\n")

    drone_takeoff(leader, target_system=1, target_altitude=-20)
    drone_takeoff(follower, target_system=2, target_altitude=-20)

    # drone_hold(leader, target_system=1)
    # drone_hold(follower, target_system=2)

    print("\n")

    drone_land(leader, target_system=1)
    drone_land(follower, target_system=2)

if __name__ == "__main__":
    main()
