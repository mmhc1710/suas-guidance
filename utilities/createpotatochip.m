clear all
close all
clc


theta = linspace(0,2*pi,500);
x = 750*cos(theta);
y = 400*sin(theta);
z = -300 + -125*cos(2*theta);

figure
plot3(x,y,z)
axis equal
grid on
M = [x',y',z'];

csvwrite('potatochip.csv',M)