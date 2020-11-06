clear;clc;close all;

% Load gyro data
load('sensorData.mat')

p = table2array(AngularVelocity(:,1));
q = table2array(AngularVelocity(:,2));
r = table2array(AngularVelocity(:,3));
t = transpose(linspace(0,100,length(p)));

% Define initial conditions for Euler angles
phi0 = 85.412321;
theta0 = -0.190273;
psi0 = 0.249425;