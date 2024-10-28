from dronekit import connect, VehicleMode , LocationGlobalRelative
import time

uav = connect('127.0.0.1:14550', wait_ready=True)

# print(f"Armable: {uav.is_armable}")
# print(f"Armed: {uav.armed}")
# At stabilize mode the drone rotated by remote controller.
# So, we have to take the mode of the drone to guided in order to move the uav by code.
# After changing the mode we can arm the uav easily changing the uav.armed variable of uav
# if uav.is_armable:
#     uav.mode = VehicleMode("GUIDED")
#     uav.armed = True
#     uav.simple_takeoff(10)
# else:
#     print(f"Unable to arm uav : Armable = {uav.is_armable}")


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
# You shouldn't put lots of time.sleep() function between arm function and takeoff
# Because uav is arming itself at so close time and disarming as well in a so close time
# And then trying to takeoff at the disarmed position.

takeoff(10)
print("Position is successfully accepted")
position = LocationGlobalRelative(-35.36227792, 149.16508368, 30)
uav.simple_goto(position)