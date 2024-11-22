from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
'''
Step 1: Takeoff and Navigate to a Waypoint
    1.1 Add a takeoff command.
    1.2 Set a waypoint for the drone to fly toward.
    1.3 Once it reaches the waypoint, change the flight mode to LAND.
    
Step 2: Simulate Obstacle Detection


'''

uav = connect('127.0.0.1:14550')
# Initialize parameters
uav.parameters['RNGFND1_TYPE'] = 1
uav.parameters['RNGFND1_MIN_CM'] = 1000
uav.parameters['RNGFND1_MAX_CM'] = 20

# 1.1
def takeoff(altitude):
    while uav.is_armable is False:
        print("INFO: UAV is not armable.")
        time.sleep(1)
    print("INFO: UAV is armable.")

    uav.mode = VehicleMode("GUIDED")
    while uav.mode != "GUIDED":
        print("Waiting for mode change to GUIDED...")
        time.sleep(1)

    uav.armed = True
    start_time = time.time()
    while uav.armed is False and time.time() - start_time < 30:
        print("PROCESS: UAV is arming.")
        time.sleep(1)
    if uav.armed == False:
        print("ERROR: UAV failed to arm.")
        return
    print("INFO: UAV successfull armed.")

    uav.simple_takeoff(altitude)
    time.sleep(2)
    
    while uav.location.global_relative_frame.alt <= altitude * 0.9:
        print("PROCESS: UAV is taking off.")
        time.sleep(1)
    print("INFO: Takeoff successful.")

def mission():
    global command

    command = uav.commands

    command.clear()
    time.sleep(1)
    # TAKEOFF
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)) # it counts this as zero and doesn't implementing
    # WAYPOINT
    command.add(Command(0, 0, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, -35.36237003, 149.16573203, 20))
    # LAND
    command.add(Command(0, 0, 2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 0))
    # Check
    command.add(Command(0, 0, 3, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    command.upload()
    print("INFO: Commands uploading...")

    command.wait_ready()  # Ensure that the commands are ready
    if len(command) > 0:
        print(f"INFO: {len(command)} mission commands uploaded successfully.")
    else:
        print("ERROR: Mission upload failed.")

def avoid_obstacle():
    """
    This function handles obstacle avoidance by moving the drone in a
    small detour to avoid the obstacle.
    """
    # For simplicity, we'll move the drone 5 meters to the right and up.
    print("INFO: Avoiding obstacle...")

    # Get current location
    current_location = uav.location.global_relative_frame

    # Move the drone 5 meters to the right (east) and 5 meters up (higher altitude)
    new_location = LocationGlobalRelative(current_location.lat, current_location.lon + 0.00005, current_location.alt + 5)
    uav.simple_goto(new_location)

    # Wait for the drone to reach the new location
    time.sleep(5)  # Adjust time depending on the distance

    print("INFO: Obstacle avoided. Resuming mission...")

#3
takeoff(10)

mission()
command.next = 0

uav.mode = VehicleMode("AUTO")
while uav.mode != 'AUTO':
    print("Waiting for mode change to AUTO...")
    time.sleep(1)

while True:
    next_waypoint = uav.commands.next
    distance = uav.rangefinder.distance

    if distance <= 5:
        print(f"Obstacle detected at {distance} meters! Stopping UAV.")
        uav.simple_goto(uav.location.global_frame)  # Hover in place
        avoid_obstacle()
    print(f"Next command: {next_waypoint}")
    print(f"Current location: {uav.location.global_relative_frame}")
    print(f"Current mode: {uav.mode}")
    time.sleep(1)

    if next_waypoint >= len(command) - 1:  # Once the last command is reached
        print("Final waypoint reached, mission complete.")
        break

print("While is end")