#
# This is a small example of the python drone API
# Usage:
# * mavproxy.py
# * module load api
# * api start small-demo.py
#
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import math
import time

# First get an instance of the API endpoint
api = local_connect()
# get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
v = api.get_vehicles()[0]

# Now change the vehicle into fbwa mode
print "Switching to AUTOTUNE Mode"
v.mode = VehicleMode("AUTOTUNE")

print "Overriding a RC channel"
for i in range(0,30):
	v.channel_override = {"1" : 1500, "2" : 1000, "3" : 1500}
	v.flush()
	time.sleep(2)
	v.channel_override = {"1" : 1500, "2" : 2000, "3" : 1500}
	v.flush()
	time.sleep(2)

for i in range(0,30):
	v.channel_override = {"1" : 1000, "2" : 1500, "3" : 1500}
	v.flush()
	time.sleep(2)
	v.channel_override = {"1" : 2000, "2" : 1500, "3" : 1500}
	v.flush()
	time.sleep(2)

print "Cancel RC override"
v.channel_override = {"1" : 0, "2" : 0, "3" : 0}
v.flush()

print "Reverting to AUTO mode"
v.mode = VehicleMode("AUTO")

print "New gains!"
print "Roll2Servo P: %s" % v.parameters['RLL2SRV_P'],
print "Roll2Servo I: %s" % v.parameters['RLL2SRV_I'],
print "Roll2Servo D: %s" % v.parameters['RLL2SRV_D'],
print "Pitch2Servo P: %s" % v.parameters['PTCH2SRV_P'],
print "Pitch2Servo I: %s" % v.parameters['PTCH2SRV_I'],
print "Pitch2Servo D: %s" % v.parameters['PTCH2SRV_D'],

# Always call flush to guarantee that previous writes to the vehicle have taken place
v.flush()
