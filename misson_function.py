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
    # This code line helps us to reach any command that exist in the drone currently, or add and chnage them if needed.
    command = uav.commands
    # With this command we can force the robot to clear if any mission exist/added before.
    command.clear()
    time.sleep(1)

    command.add(Command())