from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

'''
Algorithm
1. Connect Drone
2. Arm Drone 
3. Takeoff Drone
4. Load missions to drone
5. Finish
'''
#1
uav = connect('127.0.0.1:14550', wait_ready=True)


def takeoff(altitude):
    while uav.is_armable is not True:
        print("INFO: UAV is not armable.")
        time.sleep(1)
    print("INFO: UAV is armable.")
    uav.mode = VehicleMode("GUIDED")
    while uav.mode != 'GUIDED':
        print("Waiting for mode change to GUIDED...")
        time.sleep(1)

    uav.armed = True

    start_time = time.time()
    while not uav.armed and time.time() - start_time < 30:  # Timeout of 30 seconds
        print("Waiting for UAV to arm...")
        time.sleep(1)
    if not uav.armed:
        print("ERROR: UAV failed to arm.")
        return
    
    print(f"CONDITION: UAV armed: {uav.armed}")

    uav.simple_takeoff(altitude)
    time.sleep(1)
    print("INFO: Takeoff started.")
    while uav.location.global_relative_frame.alt < altitude * 0.9:
        print("PROCESS: Taking off.")
        time.sleep(0.5)

    print("INFO: Takeoff successful.")

def mission():
    global command

    command = uav.commands

    command.clear()
    time.sleep(1)
    # TAKEOFF
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    # WAYPOINT
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36220327, 149.16508102, 30))
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36321952, 149.16574906, 50))

    # RTL
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    # Check
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    command.upload()
    print("INFO: Commands uploading...")

    command.wait_ready()  # Ensure that the commands are ready
    if len(command) > 0:
        print(f"INFO: {len(command)} mission commands uploaded successfully.")
    else:
        print("ERROR: Mission upload failed.")

#3
takeoff(10)

mission()
command.next = 0

uav.mode = VehicleMode("AUTO")
while uav.mode != 'AUTO':
    print("Waiting for mode change to AUTO...")
    time.sleep(1)

while True:
    next_waypoint = command.next
    print(f"Next command {next_waypoint}")
    time.sleep(1)

    if next_waypoint == len(command):
        print("Misson complete.")
        break

print("While is end")