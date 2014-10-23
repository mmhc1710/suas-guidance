function [x,y,z] = lla2flatdumb(lat,lon,alt,lat0,lon0,alt0)
re = 6378137;
re_c = re*cos((pi/180)*abs(lat0));
x = (lon-lon0)*(re_c*pi)/180;
y = (lat-lat0)*(re*pi)/180;
z = alt-alt0;
