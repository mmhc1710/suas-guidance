import math
import csv
import numpy
import matplotlib.pyplot as plt
import FindInt
import time
def Angle_wrap(eta):
	if numpy.absolute(eta) > numpy.pi:
		if eta>0:
			eta = eta -2*numpy.pi
		elif eta<0:
			eta = eta +2*numpy.pi
	return eta

def Guide(Pathx,Pathy,Pathz,ACx,ACy,ACz,ACchi,ACv,MinIndex = -1):
#NED!!!!!!!!!!!!!!!!!!!!!

	######################################## Define Gain ##################################################
	L1 = 140#meters
	#######################################################################################################

	#Define north vector
	North =[]
	North.append(1)
	North.append(0)

	#Calculate closest point to path
	Dist = []
	PX = numpy.array(Pathx)
	PY = numpy.array(Pathy)

	Dist = numpy.sqrt((PX-ACx)**2 + (PY-ACy)**2)


	if MinIndex == -1:
		MinIndex = numpy.argmin(Dist)#The closest point is the one with the minimum distance
	MinIndex1 = numpy.argmin(Dist)

	if Dist[MinIndex1] < L1*1.2:
		MinIndex = numpy.argmin(Dist)#The closest point is the one with the minimum distance
		#print 'are we slow?'
		#find the indicies of all points sorted by 
		dist_inds = FindInt.dist_inds(L1,Pathx,Pathy,ACx,ACy,ACz,ACchi)
		dist_inds = dist_inds[range(20)]
		#number of points
		length = dist_inds.shape[0] #MAKE SURE THAT dist_inds is a 1D array
		#find the difference in the indicies from the index of the closest point
		ind_diff_list = dist_inds - MinIndex
		#find where that difference is above 0
		ind_diff_log = ind_diff_list > 0
		#trim to only those points
		ind_diff_pos = ind_diff_list[ind_diff_log]
		#if there are all positive or no positive index differences we are near the break and should take the minimum difference
		#print ind_diff_pos
		if ind_diff_pos.size == 0 or ind_diff_pos.size == length: 
			index_ref = min(ind_diff_list) + MinIndex
		#otherwise pick the smallest positive increase of the 10 points closest to L1
		else:
			if ind_diff_pos.size > 10:
				r = 10
			else:
				r = ind_diff_pos.size
			ind_diff_pos_trun = ind_diff_pos[range(r)]#may have to tune this
			ind_diff_pos_sort = numpy.sort(ind_diff_pos_trun)
			index_ref = ind_diff_pos_sort[0] + MinIndex
	else:
		index_ref = MinIndex#If we are further than L1 away from any point on the path, use the closest point as the 	

	#index_ref = 0
	#print Dist[index_ref]
	Refx = Pathx[index_ref]#referece point
	Refy = Pathy[index_ref]

	#Solve for nonlinear logic parameters
	L1Vec = []#Get the L1 vector
	L1Vec.append(Refx - ACx)
	L1Vec.append(Refy - ACy)

	#Find the angle of the L1 vector with respect to north (ie the course angle of the L1 vector)
	L1Angle = math.atan2(L1Vec[1],L1Vec[0])#math.acos(numpy.dot(North,L1Vec)/(numpy.linalg.norm(North,2)*numpy.linalg.norm(L1Vec,2)))

	#Find the angle eta = the difference between the L1 vector angle and the current course angle
	eta = L1Angle - ACchi	
	if eta>numpy.pi:
		eta = eta-2*numpy.pi
	if eta<-numpy.pi:
		eta = eta+2*numpy.pi

	#Find the instantaneous radius of the path according 
	R = L1/(2*numpy.sin(eta))

	ChiDotDesired = ACv/R

	DesiredAltitude = Pathz[MinIndex1]
	return ChiDotDesired, DesiredAltitude, MinIndex
