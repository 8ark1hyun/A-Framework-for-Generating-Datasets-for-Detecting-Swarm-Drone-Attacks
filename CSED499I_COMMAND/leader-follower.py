import time
import math
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
    print(f"[System ID: {target_system}] Drone Disarmed!")

    time.sleep(5)

def drone_position(drone):
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1e3
        return latitude, longitude, altitude
    return None, None, None

def drone_takeoff(drone, target_system):
    print(f"[System ID: {target_system}] Taking off...")
    drone.mav.command_long_send(
        target_system,
        0,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 10
    )
    print(f"[System ID: {target_system}] Takeoff!")

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 10) > 1:
            continue
        else:
            time.sleep(2)
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
            time.sleep(2)
            return

def calculate_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371e3
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    bearing = math.degrees(math.atan2(
        math.sin(dlon) * math.cos(lat2_rad),
        math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    ))

    return distance, (bearing + 360) % 360

def smooth_velocity(distance, target_distance, max_speed):
    if distance <= target_distance:
        return 0
    error = distance - target_distance
    velocity = max_speed * min(error / target_distance, 1)
    return velocity

def send_velocity(follower, vx, vy, vz):
    follower.mav.set_position_target_local_ned_send(
        0,
        2,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0,
    )

def main():
    leader = drone_connect(14540)
    follower = drone_connect(14541)

    drone_arm(leader, target_system=1)
    drone_arm(follower, target_system=2)

    drone_takeoff(leader, target_system=1)
    drone_takeoff(follower, target_system=2)

    desired_distance = 2
    max_speed = 10.0

    while True:
        leader_position = drone_position(leader)
        follower_position = drone_position(follower)

        distance, bearing = calculate_distance_and_bearing(leader_position[0], leader_position[1], follower_position[0], follower_position[1])

        speed = smooth_velocity(distance, desired_distance, max_speed)
        vx = speed * math.cos(math.radians(bearing))
        vy = speed * math.sin(math.radians(bearing))

        send_velocity(follower, vx, vy, 0)

        time.sleep(0.1)


    drone_land(leader, target_system=1)
    drone_land(follower, target_system=2)

    drone_disarm(leader, target_system=1)
    drone_disarm(follower, target_system=2)

    leader.close()
    follower.close()

if __name__ == "__main__":
    main()







