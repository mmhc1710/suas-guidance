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
# And the callback is still executed from the message recv thread. The right answer might include
# a condition variable and a message queue

#The point is, if you're executing something of any substance from this callback, beware of taking too much time, 
#since messages won't be received while this function is executing

def mode_callback(name):
    global shouldQuit #since this is on the main MAVProxy thread, not the independent thread we've got
    print "System status: %d" % ( v.system_status)
    if (v.system_status == 5):
        print "Detected loss of GCS link!"
        shouldQuit = True

# First get an instance of the API endpoint
api = local_connect()
# get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
v = api.get_vehicles()[0]

print "WP_Failsafe started, watching for GCS lost link"
v.add_attribute_observer("mode", mode_callback)

while not (threading.current_thread().exit or shouldQuit):   
    #until we are signalled to exit, 
    #everything is run from the callback
    time.sleep(1)

v.remove_attribute_observer("mode", mode_callback)
print "Unregistered callback, exiting"

