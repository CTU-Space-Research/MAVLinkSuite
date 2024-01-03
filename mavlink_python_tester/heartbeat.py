MAVLINK_DIALECT = 'ctu_sr_illustria_2'
MAVLINK20 = 1
from pymavlink import mavutil
#from pymavlink.dialects.v20 import ctu_sr_illustria_2 as mavutil
import time

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('/dev/tty.usbserial-A50285BI',baud=57600)

#the_connection.logfile_raw = open("log.txt", "w")

the_connection.disable_signing()
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
#for some reason it doest not read the system id and component id correctly
#the_connection.target_system = 2
#the_connection.target_component = 15


while (the_connection.target_system == 0):
    msg = the_connection.recv_match(type='HEARTBEAT',blocking=True)
    the_connection.target_system = msg._header.srcSystem
    the_connection.target_component = msg._header.srcComponent

print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

#the_connection.mav.protocol_version_send(the_connection.target_system, the_connection.target_component)

#the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0, 300, 0,0,0,0,0,0)

the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

while 1:
    the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print("sent HEARTBEAT")
    time.sleep(1.5)

while (1):
    msg = the_connection.recv_match(blocking=True)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0, 300, 0,
                                      0,0,0,0,0)
    print(msg)

while 1:
    the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print("sent HEARTBEAT")
    time.sleep(1.5)
# Once connected, use 'the_connection' to get and send messages
