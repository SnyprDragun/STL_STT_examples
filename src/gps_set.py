#!/usr/bin/env python3

from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

latitude = 473977420
longitude = -122084846
altitude = 500000

time_usec = int(time.time() * 1e6)

master.mav.gps_global_origin_send(
	0,
	latitude,
	longitude,
	altitude,
	time_usec
)
