from dronekit import connect

# Connection ip of drone in the simulation is "127.0.0.1:14550" and wait_ready=True means try to connect drone until connect operation successful
drone = connect('127.0.0.1:14550', wait_ready=True)

print(f"Drone arm condition: {drone.armed}")
# It holds an altitude which is altitude of drone from sea level
#print(f"Global frame: {drone.location.global_frame}")
# The altitude for relative frame is from ground level
#print(f"Global relative frame: {drone.location.global_relative_frame}")
# by the last one we can print only altitude
print(f"Altitude: {drone.location.global_relative_frame.alt}")