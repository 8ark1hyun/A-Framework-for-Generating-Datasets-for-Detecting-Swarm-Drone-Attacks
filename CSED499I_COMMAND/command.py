import time
from pymavlink import mavutil

def drone_connect(udp_port):
    print(f"Connecting to drone on UDP port {udp_port}...")
    drone = mavutil.mavlink_connection(f'udp:localhost:{udp_port}')

    drone.wait_heartbeat()
    print(f"Drone on port {udp_port} connected!")

    return drone
        
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
    
def drone_disarm(drone, target_system):
    print(f"[System ID: {target_system}] Disarming drone...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    drone.motors_armed_wait()
    print(f"[System ID: {target_system}] Drone Disarmed!")

    time.sleep(5)

def drone_position(drone):
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1000.0
        return latitude, longitude, altitude
    return None, None, None

def drone_takeoff(drone, target_system):
    print(f"[System ID: {target_system}] Taking off...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 30
    )
    print(f"[System ID: {target_system}] Takeoff!")

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 30) > 1:
            continue
        else:
            return

def drone_land(drone, target_system):
    print(f"[System ID: {target_system}] Landing...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 0
    )
    print(f"[System ID: {target_system}] Land!")

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 0) > 1:
            continue
        else:
            return

def main():
    leader = drone_connect(14540)
    follower = drone_connect(14541)

    print("\n")

    drone_arm(leader, target_system=1)
    drone_arm(follower, target_system=2)

    print("\n")

    drone_takeoff(leader, target_system=1)
    drone_takeoff(follower, target_system=2)

    print("\n")

    drone_land(leader, target_system=1)
    drone_land(follower, target_system=2)

    print("\n")

    drone_disarm(leader, target_system=1)
    drone_disarm(follower, target_system=2)

    leader.close()
    follower.close()

if __name__ == "__main__":
    main()
