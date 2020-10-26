clc; close all; clear;

Fb_params.line_width = 4;
Fb_params.plot_type = 1;

% Define rotation angles
psi = 70 * pi/180;
theta = 130 * pi/180;
phi = 25 * pi/180;

% Compute Angles
C1v = [cos(psi) sin(psi) 0;
    -sin(psi) cos(psi) 0;
    0 0 1];

C21 = [cos(theta) 0 -sin(theta);
    0 1 0;
    sin(theta) 0 cos(theta)];

Cb2 = [1 0 0;
    0 cos(phi) sin(phi);
    0 -sin(phi) cos(phi)];

Cbv = Cb2 * C21 * C1v;

% Get Eigenvalues and eigenvectors
[V,D] = eig(Cbv)

idxUnityEigenvalue = 3;
lambda = D(idxUnityEigenvalue, idxUnityEigenvalue);
v = V(:,idxUnityEigenvalue);

tol = 10e-6;
assert(max(abs(Cbv*v - lambda*v))<tol)

% Rotate the x-axis about v using Rodrigues
mu = linspace(0,233.5*pi/180,50);

euler_angle = [phi; theta; psi]


