#Basic example showing how to avoid disabling the failsafes
#
#Concept: Disabling MAVProxy's heartbeats means that the only source is the regular GCS
#         If the real GCS is there, HEARTBEAT msg->system_status = 4 
#         If the real GCS isn't there, HEARTBEAT msg->system_status =  5 (i.e. lost comm)
#            The waypoint follower needs to stop sending commands at this point
#
#Setup:
#
# 1. Fire up MAVProxy
# 2. Disable MAVProxy's heartbeats: 
#    set heartbeat 0
# 3. Load via DroneAPI
#    module load api
#    api start wp_failsafe.py
#
#

import threading
import time
from droneapi.lib import VehicleMode
from pymavlink import mavutil

shouldQuit = False

#This function ends up executing on the message recv thread from within DroneAPI - not the dedicated thread setup to run
# function itself
#The fix is patching DroneAPI to pass through system_status from the HEARTBEAT message

def mavrx_debug_handler(message):
    """A demo of receiving raw mavlink messages"""
    #print "Received", message
    typ = message.get_type()
    if typ == 'HEARTBEAT':
        if (message.system_status == 5):
            print "Detected loss of GCS link, exiting"
            shouldQuit = True

# First get an instance of the API endpoint
api = local_connect()
# get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
v = api.get_vehicles()[0]
v.set_mavlink_callback(mavrx_debug_handler)
print "WP_Failsafe started, watching for GCS lost link"

while not (threading.current_thread().exit or shouldQuit):   #until we are signalled to exit, everything is run from the callback
    time.sleep(1)
    print "ShouldQuit: %d" % (shouldQuit)
v.set_mavlink_callback(None)
print "Unregistered callback, exiting"

