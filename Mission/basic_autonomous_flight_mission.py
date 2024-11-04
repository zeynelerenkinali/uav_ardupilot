
from dronekit import connect

uav = connect('127.0.0.1:14550', wait_ready=True)

