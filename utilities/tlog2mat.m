clear flightpath_xyz;
temp=csvread('log.csv',1,0);
flightpath_gps = [temp(:,6)*1e-7,temp(:,7)*1e-7,temp(:,5)];
[flightpath_xyz(:,1), flightpath_xyz(:,2), flightpath_xyz(:,3)] = lla2flatdumb(flightpath_gps(:,1),flightpath_gps(:,2),flightpath_gps(:,3), 40.1447601,-105.2435532,1680.38);
save run1_insidechip flightpath_gps flightpath_xyz chip_enu;

figure
plot3(flightpath_xyz(:,1), flightpath_xyz(:,2), flightpath_xyz(:,3))
hold on
plot3(chip_enu(:,1), chip_enu(:,2), chip_enu(:,3))
axis equal

