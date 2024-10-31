from dronekit import connect, VehicleMode , LocationGlobalRelative, Command
from pymavlink import mavutil
import time

uav = connect('127.0.0.1:14550', wait_ready=True)

def takeoff(altitude):
    while uav.is_armable is not True:
        print("UAV is not armable!")
        time.sleep(1)
    print("UAV is armable.")
    uav.mode = VehicleMode("GUIDED")
    #print(f"UAV mode: {str(uav.mode)}")
    uav.armed = True

    while uav.armed is not True:
        print("UAV is arming...")
        time.sleep(0.5)
    
    print(f"UAV armed: {uav.armed}")
    uav.simple_takeoff(altitude)
    while uav.location.global_relative_frame.alt < altitude * 0.9:
        print("UAV is taking off...")
        time.sleep(0.5)

def mission_add():
    global command
    # This code line helps us to reach any command that exist in the drone currently, or add and chnage them if needed.
    command = uav.commands
    # With this command we can force the robot to clear if any mission exist/added before.
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
    print("Commands uploading...")

takeoff(10)

mission_add()

command.next = 0

uav.mode = VehicleMode("AUTO")

while True:
    next_waypoint = command.next
    print(f"Next command {next_waypoint}")
    time.sleep(1)

    if next_waypoint is 4:
        print("Mission end")
        break
    
print("While is end")
