#Controller to track a 3d path for a suas
#Usage: Place suas-guidance directory in the ardupilot/ArduPlane directory
#Usage (continued): within MAVProxy console, "api start ./suas-guidance/controller.py"
#Authors: Tevis Nichols, Will Silva, Paul Guerrie, Steve McGuire, Aaron Buysse, Matthew Aitken
#Last modified: 10/20/2014
import sys, os
currentdir = os.path.dirname(os.path.realpath("__file__")) + '/suas-guidance/'
sys.path.append(currentdir)
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import math
import time
import numpy as np
from numpy  import *
import csv
import Guide_nichols #L1 logic
from coordtrans import * #GPS to cartesian
from cmd_saturate import cmd_saturate #limits RC commands to saturation limits

###CONNECT TO AIRCRAFT (SITL)##########################################################
api = local_connect()

#establish object link to aircraft, v
v = api.get_vehicles()[0]
#######################################################################################

###SET AIRCRAFT LIMITS#################################################################
v.parameters['RC1_MAX'] = 2000
v.parameters['RC1_MIN'] = 1000
v.parameters['RC2_MAX'] = 2000
v.parameters['RC2_MIN'] = 1000
v.parameters['RC3_MAX'] = 2000
v.parameters['RC3_MIN'] = 1000
v.parameters['LIM_PITCH_MIN'] = -3000
v.parameters['LIM_PITCH_MAX'] = 3000
v.parameters['LIM_ROLL_CD'] = 6500
v.parameters['RLL2SRV_D'] = 0.137
v.parameters['RLL2SRV_I'] = 0.133
v.parameters['RLL2SRV_P'] = 1.961
v.parameters['PTCH2SRV_D'] = 0.232
v.parameters['PTCH2SRV_I'] = 0.207
v.parameters['PTCH2SRV_P'] = 3.321
########################################################################################

###IMPORT CSV PATH######################################################################
#Read In Path
Reader = csv.reader(open(currentdir + 'potatochip.csv', 'rv'))#create reader object
ind = 0#initialize ind, Pathx, Pathy and Pathz
Pathx = []
Pathy = []
Pathz = []
#for all the rows in the csv file, assign each datum to its appropriate variable
for row in Reader:
	Pathx.append(float(row[0]))#NED!!!!!!!!!!!!!!!!!!!!
	Pathy.append(float(row[1]))
	Pathz.append(float(row[2]))
	ind = ind + 1
########################################################################################

###RUN-TIME CONSTANTS###################################################################

#constants
pitch_trim = 0.989 #deg
throttle_trim = 1230 #RC channel
start_alt = 1680.38 #m
g = 9.81 #m/s^2
speed_desired = 22 #m/s

#desired coordinate center (datum)
lat0 = 40.1447601
lon0 = -105.2435532
alt0 = 1680.38

# gains
KP_pitch = 1.0#0.3#Proportional gain for altitude to pitch
KI_pitch = 0.3#0.03#Integral gain for altitude to pitch
KD_pitch = 0.8#0.6#Derivative gain for vertical velocity to pitch
KPR_pitch = 0.0 #Pitch Rate gain, currently unused

KP_throttle = 4.0
KD_throttle = 1.5 #currently unused
KFF_throttle = 350.0#20.0

KP_roll = 13.0 #deg/(rad/s)

#altitude integrator wind-up limit
alt_int_max = 100

RC1_MAX = v.parameters['RC1_MAX']
RC1_MIN = v.parameters['RC1_MIN']
RC1_ZERO = 1500;

RC2_MAX = v.parameters['RC2_MAX']
RC2_MIN = v.parameters['RC2_MIN']
RC2_ZERO = 1500;

RC3_MAX = v.parameters['RC3_MAX']
RC3_MIN = v.parameters['RC3_MIN']
RC3_ZERO = 1500;

LIM_PITCH_MIN = v.parameters['LIM_PITCH_MIN']
LIM_PITCH_MAX = v.parameters['LIM_PITCH_MAX']

LIM_ROLL  = v.parameters['LIM_ROLL_CD']

# initialize integrator
prev = time.time()
alt_int_err = 0

# Now change the vehicle into fbwa mode
print "Switching to FBWA Mode"
v.mode = VehicleMode("FBWA")

initial_t = time.time()
flag1 = True
loopcount = 0
while True:
	rollspeed = v.angularRates[0]
	pitchspeed = v.angularRates[1]
	yawspeed = v.angularRates[2]
	#print 'start'
	###GPS COORDINATE TRANSFORM#################################################################
	flat = lla2flatdumb(v.location.lat,v.location.lon,v.location.alt,lat0,lon0,alt0)
	X = flat[0] #ENU
	Y = flat[1] #ENU
	Z = flat[2] #ENU
	#print Z
	#print [Y, X, -Z]
	if (v.velocity[0] > 0) or (v.velocity[0] < 0): #may want to set this threshold higher at some point
		Chi = np.arctan2(v.velocity[1],v.velocity[0]) #assuming zero path angle is aligned with x
	else:
		Chi = 0;
	#print'Chi'
	#print Chi*180/pi
	############################################################################################
	
	###PATH FOLLOWING LOGIC (L1 GUIDANCE)############################################
	#if loopcount %500 == 0:
	if flag1:	
		[turn_rate_des,z_target,MinIndex] = Guide_nichols.Guide(Pathx,Pathy,Pathz,Y,X,-Z,Chi,speed_desired) #NED
		flag1 = False
	else:
		[turn_rate_des,z_target,Minjunk] = Guide_nichols.Guide(Pathx,Pathy,Pathz,Y,X,-Z,Chi,speed_desired,MinIndex)
	turn_rate_des = cmd_saturate(turn_rate_des,0.3,-0.3) #NED
	alt_target = alt0 - z_target #altitude positive but z down
	#print Z + z_target
	#for holding turn rate/altitude, use lines below instead
	#turn_rate_des = 0*pi/(180) #NED turn rate (positive CW)
	#alt_target = start_alt + (time.time()-initial_t)*5 #ENU alt (positive up)
	#############################################################################################

	###TURN RATE CONTROLLER######################################################################

	#calculate bank angle required for radius at a certain airspeed

	#TODO would like to know angular rates (where are these in v?) in order to close turn rate 		control loop and add pitch rate damping to altitude control

	#a = turn_rate_des*v.airspeed 
	#print 'desired turn'
	#print turn_rate_des
	bankangle_r = np.arctan((turn_rate_des*v.airspeed)/g) 
	#print 'bank'
	#print bankangle_r*180/np.pi
	theta = v.attitude.pitch
	phi = v.attitude.roll
	psi = v.attitude.yaw
	Rib = array([[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)], [sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta)], [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)]])

	Rbi = np.transpose(Rib)
	omega = array([rollspeed, pitchspeed, yawspeed]).reshape(3,1) #need to know how to get rates
	euler_rate = dot(Rbi,omega)
	turn_rate = euler_rate[2][0] #note negative sign to switch to ENU coordinates NOW STAYING IN NED
	turn_rate_err = turn_rate - turn_rate_des
	# positive roll -> negative turn rate
	#turn_rate_err = 0

	#convert bank angle to degrees
	bankangle_d_ff = bankangle_r * 180/math.pi
	#print bankangle_d_ff

	#add proportional term
	bankangle_d = bankangle_d_ff - KP_roll*turn_rate_err
	#print'bank'
	#print bankangle_d_ff
	#convert bank angle to RC stick value
	bankangle_stick = (50*(RC1_MAX - RC1_MIN)/(LIM_ROLL))*bankangle_d + RC1_ZERO

	RC1_cmd = bankangle_stick
	#############################################################################################

	###ALTITUDE CONTROLLER (PID with climb rate feed forward)####################################
	
	#feed forward term
	#feed forward position
	dt = 1
	X_ff = X + v.velocity[1]*dt
	Y_ff = Y + v.velocity[0]*dt
	Z_ff = Z + v.velocity[2]*dt #velocity comes out in NEU
	Chi_ff = Chi + turn_rate*dt
	
		#call control on FF position
	
	[turn_rate_ff,z_target_ff,crap] = Guide_nichols.Guide(Pathx,Pathy,Pathz,Y_ff,X_ff,-Z_ff,Chi_ff,speed_desired,MinIndex)
	alt_des_ff = alt0 - z_target_ff
	#determine climb rate needed
	climb_des = (alt_des_ff - alt_target)/dt
	
	
	time_step = time.time() - prev
	#print 'time step'
	#print time_step
	prev = time.time()
	
	if alt_target - v.location.alt < 20 and alt_target - v.location.alt > -25:
		alt_int_err += time_step*( v.location.alt - (alt_target))
	else:
		alt_int_err = 0
	if np.absolute(climb_des) < 1.5: #np.absolute(alt_target - v.location.alt) < 10 and 
		alt_int_err = alt_int_err*0.95

	#print 'alt err'
	#print v.location.alt - (1680+alt_target)
	#print 'integrator'
	#print alt_int_err
	#print 'alt'
	#print v.location.alt
	
	if alt_int_err > alt_int_max:
		alt_int_err = alt_int_max
	if alt_int_err < -alt_int_max:
		alt_int_err = -alt_int_max
	#print 'alt target'
	#print 1680+alt_target


	#t_ff = time.time() + dt #feed forward time (if needed)
	#print X_ff - X
	#print Y_ff - Y
	#print Z_ff - Z
	#print v.airspeed



	# add all terms for altitude control
	#pitch_cmd = KP_pitch * ( alt_target - v.location.alt) - KI_pitch*alt_int_err - KD_pitch * (v.velocity[2] - climb_des)- pitch_trim - KPR_pitch*v.angularRates[1] #system NEU?!?! # 
	#print 'p cmd'
	#print pitch_cmd
	#print 'proportional'
	print [KP_pitch * ( alt_target - v.location.alt), - KI_pitch*alt_int_err, -KD_pitch * (v.velocity[2] - climb_des), climb_des]
	#print v.velocity[2]
	#print 'integral'
	#print -KI_pitch*alt_int_err
	#print 'derivative'
	#print -KD_pitch * (v.velocity[2] - climb_des)
	#print 'climb des'
	#print climb_des
	#print 'chi'
	#print Chi
	#print 'alt_target'
	#print alt_target
	#print 'alt_target_ff'
	#print alt_des_ff
	#print 'turn_rate'
	#print turn_rate
	#print climb_des
	#print 'end'
	###Convert pitch_cmd to RC2 value###
	if pitch_cmd > 0:
		RC2_cmd = -pitch_cmd * (RC2_ZERO - RC2_MIN)/(LIM_PITCH_MAX)*100 + RC2_ZERO
	elif pitch_cmd < 0:
		RC2_cmd = -pitch_cmd * (RC2_MAX - RC2_ZERO)/(-LIM_PITCH_MIN)*100 + RC2_ZERO
	else:
		RC2_cmd = RC2_ZERO
	#############################################################################################

	###SPEED CONTROLLER (P with feed forward for climb)##########################################

	#as_acc = (v.airspeed - vel_prev)/time_step
	#print as_acc

	#vel_prev = v.airspeed

	#throttle_cmd = KP_throttle * (speed_desired - v.airspeed) - KD_throttle*as_acc
	throttle_cmd = KP_throttle * (speed_desired - v.airspeed) + KFF_throttle*np.sin(v.attitude.pitch-pitch_trim*(pi/180))
	###Convert throttle_cmd to RC3 value###
	RC3_cmd = throttle_cmd * 10 + throttle_trim
	#############################################################################################

	#saturation to avoid "error: ushort format requires 0 <= number <= USHRT_MAX"
	RC1_cmd = cmd_saturate(RC1_cmd,RC1_MAX,RC1_MIN)
	RC2_cmd = cmd_saturate(RC2_cmd,RC2_MAX,RC2_MIN)
	RC3_cmd = cmd_saturate(RC3_cmd,RC3_MAX,RC3_MIN)

	#print "Overriding RC channels..."
	v.channel_override = { "1" : RC1_cmd, "2" : RC2_cmd, "3" : RC3_cmd }
	v.flush()
	loopcount = loopcount + 1
