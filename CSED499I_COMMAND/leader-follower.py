import time
import math
from pymavlink import mavutil

def drone_connect(udp_port):
    if udp_port == 14540:
        print("Connecting to Leader Drone...")
    elif udp_port > 14540:
        print("Connecting to Follower Drone...")
    drone = mavutil.mavlink_connection(f'udp:localhost:{udp_port}')

    drone.wait_heartbeat()
    if udp_port == 14540:
        print("Leader Drone Connected!")
    elif udp_port > 14540:
        print("Follower Drone Connected!")

    return drone

def drone_arm(drone, target_system):
    if target_system == 1:
        print("Arming Leader Drone...")
    elif target_system > 1:
        print("Arming Follower Drone...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    drone.motors_armed_wait()
    if target_system == 1:
        print("Leader Drone Armed!")
    elif target_system > 1:
        print("Follower Drone Armed!")

    time.sleep(5)

def drone_disarm(drone, target_system):
    if target_system == 1:
        print("Disarming Leader Drone...")
    elif target_system > 1:
        print("Disarming Follower Drone...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    if target_system == 1:
        print("Leader Drone Disarmed!")
    elif target_system > 1:
        print("Follower Drone Disarmed!")

    time.sleep(5)

def follower_set_mode(drone, target_system):
    print("Setting Follower Drone's Mode to OFFBOARD...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,
        6, 0, 0, 0, 0, 0
    )
    print("Setting Follower Drone's Mode Completed!")

def drone_position(drone):
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1e3
        return latitude, longitude, altitude
    return None, None, None

def drone_takeoff(drone, target_system):
    if target_system == 1:
        print("Taking off Leader Drone...")
    elif target_system > 1:
        print("Taking off Follower Drone...")
    current_position = drone_position(drone)

    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 10
    )

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 10) > 1:
            continue
        else:
            if target_system == 1:
                print("Leader Drone Took off!")
            elif target_system > 1:
                print("Follower Drone Took off!")
            time.sleep(2)
            return

def drone_land(drone, target_system):
    if target_system == 1:
        print("Landing Leader Drone...")
    elif target_system > 1:
        print("Landing Follower Drone...")
    current_position = drone_position(drone)

    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, lat, lon, target_alt
    )

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 0) > 1:
            continue
        else:
            if target_system == 1:
                print("Leader Drone Landed!")
            elif target_system > 1:
                print("Follower Drone Landed!")
            time.sleep(2)
            return

def calculate_position(leader_lat, leader_lon, leader_alt, distance):
    R = 6371e3
    dlat = distance / R

    target_lat = leader_lat - (dlat * 180 / math.pi)
    target_lon = leader_lon
    target_alt = leader_alt
    return target_lat, target_lon, target_alt

def send_command(follower, lat, lon, alt):
    follower.mav.set_position_target_global_int_send(
        0,
        2,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0,
        0, 0, 0,
        float('nan'), float('nan')
    )

def main():
    leader = drone_connect(14540)
    follower = drone_connect(14541)

    drone_arm(leader, target_system=1)
    drone_arm(follower, target_system=2)

    drone_takeoff(leader, target_system=1)
    drone_takeoff(follower, target_system=2)

    follower_set_mode(follower, target_system=2)

    while True:
        leader_position = drone_position(leader)

        target_lat, target_lon, target_alt = calculate_position(leader_position[0], leader_position[1], leader_position[2], distance=2.0)

        send_command(follower, target_lat, target_lon, target_alt)

        time.sleep(0.1)


    # drone_land(leader, target_system=1)
    # drone_land(follower, target_system=2)

    # drone_disarm(leader, target_system=1)
    # drone_disarm(follower, target_system=2)

    # leader.close()
    # follower.close()

if __name__ == "__main__":
    main()

