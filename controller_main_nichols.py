#Controller to track a 3d path for a suas
#Usage: (in droneproxy) api start /home/nichols/suas-guidance/controller_main_nichols.py
#Authors: Tevis Nichols, Will Silva, Paul Guerrie, Steve McGuire, Aaron Buysse, Matthew Aitken
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import math
import time
import numpy as np
from numpy  import *

def cmd_saturate(cmd,cmd_max,cmd_min):
	if cmd > cmd_max:
		cmd = cmd_max
	elif cmd < cmd_min:
		cmd = cmd_min
	return cmd

 #convert postion at lat lon alt to flat ENU coordinates centered at lat0 lon0 alt0
def lla2flat(lat,lon,alt,lat0,lon0,alt0):
  
    a = 6378137.0
    f = 1/298.257223563
    #b = a*(1-f)
    #pi = pi;
    
    e = sqrt(f*(2-f))
    
    N = a/(sqrt(1-e**2*(sin(lat))**2))
    N0 = a/(sqrt(1-e**2*(sin(lat))**2))
    
    x_ec =(alt + N)*cos(lat)*cos(lon) 
    y_ec =(alt + N)*cos(lat)*sin(lon)
    z_ec =(alt + (1-e**2)*N)*sin(lat)
    
    x_ec0 =(alt0 + N0)*cos(lat0)*cos(lon0) 
    y_ec0 =(alt0 + N0)*cos(lat0)*sin(lon0)
    z_ec0 =(alt0 + (1-e**2)*N)*sin(lat0)
    
    X_ec = array([x_ec,y_ec,z_ec]).reshape(3,1)
    X0_ec = array([x_ec0,y_ec0,z_ec0]).reshape(3,1)
    
    Xp = X_ec - X0_ec
    
    Rte = array([[-sin(lon0), cos(lon0),0],
                 [-cos(lon0)*sin(lat0), -sin(lat0)*sin(lon0), cos(lat0)],
                [cos(lat0)*cos(lon0), cos(lat0)*sin(lon0), sin(lat0)]])
 
    X_flat = dot(Rte,Xp)
    
    x = X_flat[0][0]
    y = X_flat[1][0]
    z = X_flat[2][0]
    return (x,y,z)


# First get an instance of the API endpoint
api = local_connect()

# get our vehicle - when running with mavproxy it only knows about one vehicle (for now)
v = api.get_vehicles()[0]

#Set parameters
v.parameters['RC1_MAX'] = 2000
v.parameters['RC1_MIN'] = 1000
v.parameters['RC2_MAX'] = 2000
v.parameters['RC2_MIN'] = 1000
v.parameters['RC3_MAX'] = 2000
v.parameters['RC3_MIN'] = 1000
v.parameters['LIM_PITCH_MIN'] = -2000
v.parameters['LIM_PITCH_MAX'] = 2500
v.parameters['LIM_ROLL_CD'] = 6500
v.parameters['RLL2SRV_D'] = 0.137
v.parameters['RLL2SRV_I'] = 0.133
v.parameters['RLL2SRV_P'] = 1.961
v.parameters['PTCH2SRV_D'] = 0.232
v.parameters['PTCH2SRV_I'] = 0.207
v.parameters['PTCH2SRV_P'] = 3.321

###CONTROLLER VALUES###

#constants

pitch_trim = 0.989 #deg
throttle_trim = 1230 #RC channel
start_alt = 1680 #m
g = 9.81 #m/s^2

#desired coordinate center

lat0 = -105.246
lon0 = 40.1358
alt0 = 1680

# gains
KP_pitch = 0.3#Proportional gain for altitude to pitch
KI_pitch = 0.03#Integral gain for altitude to pitch
KD_pitch = 0.6#Derivative gain for vertical velocity to pitch
KPR_pitch = 0.0 #Pitch Rate gain, currently unused

KP_throttle = 4.0
KD_throttle = 1.5 #currently unused
KFF_throttle = 3.0

KP_roll = 60.0 #deg/(rad/s)

#altitude integrator wind-up limit
alt_int_max = 140

# Now change the vehicle into fbwa mode
print "Switching to FBWA Mode"
v.mode = VehicleMode("FBWA")

RC1_MAX = v.parameters['RC1_MAX']
RC1_MIN = v.parameters['RC1_MIN']
RC1_ZERO = 1500;

RC2_MAX = v.parameters['RC2_MAX']
RC2_MIN = v.parameters['RC2_MIN']
RC2_ZERO = 1500;

RC3_MAX = v.parameters['RC3_MAX']
RC3_MIN = v.parameters['RC3_MIN']
RC3_ZERO = 1500;

LIM_PITCH_MIN  = v.parameters['LIM_PITCH_MIN']
LIM_PITCH_MAX  = v.parameters['LIM_PITCH_MAX']

LIM_ROLL  = v.parameters['LIM_ROLL_CD']

# get our vehicle state
v = api.get_vehicles()[0]

# initialize integrator
prev = time.time()
alt_int_err = 0

while (1):

	###GPS COORDINATE TRANSFORM###
	flat = lla2flat(v.location.lat,v.location.lon,v.location.alt,lat0,lon0,alt0)
	X = flat[0]
	Y = flat[1]
	Z = flat[2]
	if (v.velocity[0] > 0) or (v.velocity[0] < 0): #may want to set this threshold higher at some point
		Gamma = np.arctan(v.velocity[1]/v.velocity[0]) #assuming zero path angle is aligned with x
	else:
		Gamma = 0;
	
	###PATH FOLLOWING CONTROLLER###	
	
	turn_rate_des = 0 #ENU turn rate (positive CCW)
	alt_target = start_alt + 200 #ENU alt (positive up)
	speed_desired = 22

	###TURN RATE CONTROLLER###

	#calculate bank angle required for radius at a certain airspeed

	#TODO would like to know angular rates (where are these in v?) in order to close turn rate 		control loop and add pitch rate damping to altitude control

	#a = turn_rate_des*v.airspeed 
	
	bankangle_r = np.arctan((-turn_rate_des*v.airspeed)/g) #assumes turn rate in rad/s, negative sign to account for turn rate being expressed in ENU
	theta = v.attitude.pitch
	phi = v.attitude.roll
	psi = v.attitude.yaw
	Rib = array([[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)], [sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta)], [cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)]])

	Rbi = np.transpose(Rib)
	#omega = array([v.rates.rollspeed,v.rates.pitchspeed,v.rates.yawspeed]).reshape(3,1) #need to know how to get rates
	#euler_rate = Rbi*omega
	#turn_rate = -euler_rate[2] #note negative sign to switch to ENU coordinates
	#turn_rate_err = turn_rate - turn_rate_des
	# positive roll -> negative turn rate
	turn_rate_err = 0

	#convert bank angle to degrees
	bankangle_d_ff = bankangle_r * 180/math.pi
	#print bankangle_d_ff

	#add proportional term
	bankangle_d = bankangle_d_ff + KP_roll*turn_rate_err

	#convert bank angle to RC stick value
	bankangle_stick = (50*(RC1_MAX - RC1_MIN)/(LIM_ROLL))*bankangle_d + RC1_ZERO

	RC1_cmd = bankangle_stick

	###Altitude Controller (PID with climb rate feed forward)###

	time_step = time.time() - prev
	#print 'time step'
	#print time_step
	prev = time.time()

	alt_int_err += time_step*( v.location.alt - (alt_target))
	
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
	
	#feed forward term
	#feed forward position
	dt = 0.01
	X_ff = X + v.airspeed*np.cos(Gamma)*dt
	Y_ff = Y + v.airspeed*np.sin(Gamma)*dt
	Z_ff = Z + v.velocity[0]*dt
	#t_ff = time.time() + dt #feed forward time (if needed)
	
	#call control on FF position
	
	alt_des_ff = alt_target

	#determine climb rate needed
	climb_des = (alt_des_ff - alt_target)/dt

	# add all terms for altitude control
	pitch_cmd = KP_pitch * ( alt_target - v.location.alt) - KI_pitch*alt_int_err - KD_pitch * (v.velocity[2] - climb_des)- pitch_trim #system NEU?!?! # - KPR_pitch*v.rates.pitchspeed
	#print 'p cmd'
	#print pitch_cmd
	#print 'proportional'
	#print KP_pitch * ( alt_target - v.location.alt) 
	#print 'integral'
	#print -KI_pitch*alt_int_err
	#print 'derivative'
	#print -KD_pitch * (v.velocity[2] - climb_des)
	#print climb_des

	###Convert pitch_cmd to RC2 value###
	if pitch_cmd > 0:
		RC2_cmd = -pitch_cmd * (RC2_ZERO - RC2_MIN)/(LIM_PITCH_MAX)*100 + RC2_ZERO
	elif pitch_cmd < 0:
		RC2_cmd = -pitch_cmd * (RC2_MAX - RC2_ZERO)/(-LIM_PITCH_MIN)*100 + RC2_ZERO
	else:
		RC2_cmd = RC2_ZERO


	###Speed Controller (P with feed forward for climb)###

	#as_acc = (v.airspeed - vel_prev)/time_step
	#print as_acc

	#vel_prev = v.airspeed

	#throttle_cmd = KP_throttle * (speed_desired - v.airspeed) - KD_throttle*as_acc
	throttle_cmd = KP_throttle * (speed_desired - v.airspeed) + KFF_throttle*np.sin(v.attitude.pitch-pitch_trim)

	###Convert throttle_cmd to RC3 value###
	RC3_cmd = throttle_cmd * 10 + throttle_trim
	#print RC1_cmd
	#print RC2_cmd
	#print RC3_cmd

	#saturation to avoid "error: ushort format requires 0 <= number <= USHRT_MAX"

	RC1_cmd = cmd_saturate(RC1_cmd,RC1_MAX,RC1_MIN)
	RC2_cmd = cmd_saturate(RC2_cmd,RC2_MAX,RC2_MIN)
	RC3_cmd = cmd_saturate(RC3_cmd,RC3_MAX,RC3_MIN)

	#print "Overriding RC channels..."
	v.channel_override = { "1" : RC1_cmd, "2" : RC2_cmd, "3" : RC3_cmd }
	v.flush()


print "Current overrides are:", v.channel_override

print "RC readback:", v.channel_readback

# To Cancel override send 0 to the channels
print "Cancelling override"
v.channel_override = { "1" : 0, "4" : 0 }
v.flush()

# Now change the vehicle into auto mode
v.mode = VehicleMode("AUTO")

# Always call flush to guarantee that previous writes to the vehicle have taken place
v.flush()
