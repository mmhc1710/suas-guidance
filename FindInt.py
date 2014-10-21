def FI(L1,Pathx,Pathy,ACStatex,ACStatey,ACStatez,ACStatechi):
    import math
    import numpy
    from scipy.signal import argrelextrema
    import matplotlib.pyplot as plt
    #Define the L1 Circle and find its intersections with the path
    L1x = []
    L1y = []
    Dist = []
    theta = numpy.linspace(0,(2*math.pi),1000)
    #For every point on the L1 circle...
    for th in range(len(theta)):
        L1x.append(ACStatex + L1*math.cos(theta[th]))
        L1y.append(ACStatey + L1*math.sin(theta[th]))
	temp = []
        #For every point on the path...
        for i in range(len(Pathx)):
            temp.append(math.sqrt(math.pow((L1x[th]-Pathx[i]),2) + math.pow((L1y[th]-Pathy[i]),2)))#Calcualte the distance between each point on the path and the L1[th] point on the circle.
        Dist.append(min(temp))#the minimum of these distances is the smallest distance between that particular point on the L1 circle and the path.
    L1IntInd = numpy.argmin(Dist)
    #plt.figure(1)
    #plt.plot(L1x,L1y,'r',Pathx,Pathy)
    #plt.show()
    DistLength = len(Dist)#Book keeping for next step
    Dist.append(Dist[0])#Book keeping
    Dist.insert(0,Dist[DistLength])#Book keeping
    IntInds = [];
    TOLERANCE = 5#Intersection Tolerance = meters
    #For every minimum distance between the L1 circle points and the path
    for i in range(2,DistLength):
	if Dist[i] < Dist[i-1]:
		if Dist[i] < Dist[i+1]:
			if Dist[i] < TOLERANCE:
				IntInds.append(i)#If an element is smaller than it's neighbors, then it is a local minimum, and if it is within a certain tolerance, then it is an intersection.
    #For all the L1 circle intersection indicies, find the corresponding path indicies
    IntIndMaxPath = []
    for i in range(len(IntInds)):
    	D = []
    	for ii in range(len(Pathx)):
		D.append(math.sqrt(math.pow((L1x[IntInds[i]]-Pathx[ii]),2) + math.pow((L1y[IntInds[i]]-Pathy[ii]),2)))
    	IntIndMaxPath.append(numpy.argmin(D))
    #Find the most index positive intersection -> index of L1 circle
    IntIndMax = max(IntInds)

    if IntIndMax == len(Pathx):
	IndF = 1
	IndR = IntIndMax - 1
    else if IntIndMax == 1:
	IndF = 1 + IntIndMax
	IndR = len(Pathx)
    else:
	IndF = IntIndMax + 1
	IndR = IntIndMax - 1

    DR = math.sqrt(math.pow((L1x[L1IntInd]-Pathx[IndR]),2) + math.pow((L1y[L1IntInd]-Pathy[IndR]),2))

    INTx = Pathx[max(IntIndMaxPath)]
    INTy = Pathy[max(IntIndMaxPath)]
    print INTx, INTy
    return(INTx,INTy)
