def Guide(Pathx,Pathy,Pathz,ACx,ACy,ACz,ACchi,ACv):
    import math
    import csv
    import numpy
    import matplotlib.pyplot as plt
    import FindInt

    ######################################## Define Gain ##################################################
    L1 = 20#meters
    #######################################################################################################

    #Define north vector
    North =[]
    North.append(1)
    North.append(0)

    #Calculate closest point to path
    Dist = []
    for i in range(len(Pathx)):#For every point on the path, calculate the distance between the AC and the path point
        Dist.append(math.sqrt(math.pow((ACx-Pathx[i]),2) + math.pow((ACy-Pathy[i]),2) + math.pow((ACz-Pathz[i]),2)))

    MinIndex = numpy.argmin(Dist)#The closest point is the one with the minimum distance

    if Dist[MinIndex] < L1:
        Refx, Refy = FindInt.FI(L1,Pathx,Pathy,ACx,ACy,ACz,ACchi)
    else:
	Refx = Pathx[MinIndex]#If we are further than L1 away from any point on the path, use the closest point as the 	
	Refy = Pathy[MinIndex]#referece point

    #OPTIONAL: Plot the path, the AC and the reference point
    #plt.figure(1)
    #plt.plot(Pathx,Pathy,'b',Refx,Refy,'rx',ACx,ACy,'go')
    #plt.show()
    

    #Solve for nonlinear logic parameters
    L1Vec = []#Get the L1 vector
    L1Vec.append(Refx - ACx)
    L1Vec.append(Refy - ACy)

    #Find the angle of the L1 vector with respect to north (ie the course angle of the L1 vector)
    L1Angle = math.acos(numpy.dot(North,L1Vec)/(numpy.linalg.norm(North,2)*numpy.linalg.norm(L1Vec)))
    if L1Vec[1] < 0:
	L1Angle = 2*math.pi-L1Angle
    #Find the angle eta = the difference between the L1 vector angle and the current course angle
    eta = L1Angle - ACchi
    #Find the instantaneous radius of the path according 
    R = L1/(2*math.sin(eta))
    
    ChiDotDesired = ACv/R
    DesiredAltitude = Pathz[MinIndex]
    #print ChiDotDesired, DesiredAltitude
    return ChiDotDesired, DesiredAltitude
