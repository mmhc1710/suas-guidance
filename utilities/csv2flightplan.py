import sys, os, csv
currentdir = os.path.dirname(os.path.realpath('__file__')) + '/../'
sys.path.append(currentdir)
print currentdir
import coordtrans

#define datum
lat0 = 40.1447601
lon0 = -105.2435532
alt0 = 1680.38

Reader = csv.reader(open(currentdir + 'tilted_ellipse.csv', 'rv'))#create reader object
ind = 0#initialize ind, Pathx, Pathy and Pathz

#open write file
f = open('flightplan.txt','w')
f.write("QGC WPL 110\n")
f.write("0\t0\t0\t16\t0\t0\t0\t0\t40.1447410583496094\t-105.243423461914062\t1680.3800048828125\t1\n")
ind = ind+1
f.write("1\t0\t3\t22\t15\t0\t0\t0\t40.1447398096236228\t-105.237629413604736\t25\t1\n")
ind = ind+1

#for all the rows in the csv file, assign each datum to its appropriate variable
for row in Reader:
	(lat,lon,alt) = coordtrans.flat2lla( float(row[0]), float(row[1]), float(row[2]), lat0, lon0, alt0 )
	alt = -(alt - alt0)
	if ind % 10 == 0:
		f.write("%s\t0\t3\t16\t0\t5\t0\t0\t%s\t%s\t%s\t1\n" % (ind,lat,lon,alt))
	ind = ind + 1
f.close()
