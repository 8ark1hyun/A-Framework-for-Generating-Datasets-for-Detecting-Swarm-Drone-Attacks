# Leader Drone

import time
from pymavlink import mavutil

def drone_connect(udp_port):
    print("Connecting to Leader Drone...")
    drone = mavutil.mavlink_connection(f'udp:localhost:{udp_port}')

    drone.wait_heartbeat()
    print("Leader Drone Connected!")

    return drone

def follower_connect(udp_port):
    print("Connecting to Follower Drone...")
    follower = mavutil.mavlink_connection(f'udpout:localhost:{udp_port}')
    print("Follower Drone Connected!")

    return follower
        
def drone_arm(drone, follower):
    print("Arming Leader drone...")
    drone.mav.command_long_send(
        1,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    drone.motors_armed_wait()
    print("Leader Drone Armed!")

    time.sleep(5)
    
def drone_disarm(drone):
    print("Disarming Leader drone...")
    drone.mav.command_long_send(
        1,
        0,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Leader Drone Disarmed!")

    time.sleep(5)

def drone_position(drone):
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        latitude = msg.lat / 1e7
        longitude = msg.lon / 1e7
        altitude = msg.relative_alt / 1000.0
        return latitude, longitude, altitude
    return None, None, None

def drone_takeoff(drone):
    print("Leader Drone Taking off...")
    drone.mav.command_long_send(
        1,
        0,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 30
    )
    print("Leader Drone Takeoff!")

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 30) > 1:
            continue
        else:
            time.sleep(2)
            return

def drone_land(drone):
    print("Leader Drone Landing...")
    drone.mav.command_long_send(
        1,
        0,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 36.010550, 129.321334, 0
    )
    print("Leader Drone Land!")

    current_position = drone_position(drone)
    while True:
        current_position = drone_position(drone)
        if abs(current_position[2] - 0) > 1:
            continue
        else:
            time.sleep(2)
            return

def main():
    leader = drone_connect(14540)

    drone_arm(leader)
    drone_takeoff(leader)
    drone_land(leader)
    drone_disarm(leader)

    leader.close()

if __name__ == "__main__":
    main()
