%% Compute Linear Matrices for Extended Kalman Filter

clear;close all;clc

fp.m   = 0.7;
fp.l   = 0.3;
fp.Ixx = 1.2416;
fp.Iyy = 1.2416;
fp.Izz = 1.2416;
fp.g   = 9.81;

syms x y z psi theta phi u v w p q r
U = sym('u',[4,1]); 
X = [x;y;z;psi;theta;phi;u;v;w;p;q;r];
d = [0;0;0];%sym('d',[3,1]);

xdot = dots(X,U,d,fp);
y    = obs(X);

F = jacobian(xdot,X);
G = jacobian(xdot,U);
H = jacobian(y,X);

function dx = dots(X,U,d,fp)

x     = X(1);
y     = X(2);
z     = X(3);
psi   = X(4);
theta = X(5);
phi   = X(6);
u     = X(7);
v     = X(8);
w     = X(9);
p     = X(10);
q     = X(11);
r     = X(12);

u1 = U(1);
u2 = U(2);
u3 = U(3);
u4 = U(4);

xdot     = u;
ydot     = v;
zdot     = w;
psidot   = q*sin(phi)*sec(theta) + r*cos(phi)*sec(theta);
thetadot = q*cos(phi) - r*sin(phi);
phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);

udot     = d(1)/fp.m - u1*(1/fp.m)*(cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi));
vdot     = d(2)/fp.m - u1*(1/fp.m)*(sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi));
wdot     = d(3)/fp.m + fp.g - u1*(1/fp.m)*cos(theta)*cos(phi);

pdot     = (fp.Iyy-fp.Izz)/fp.Ixx*q*r + u2*fp.l/fp.Ixx;
qdot     = (fp.Izz-fp.Ixx)/fp.Iyy*p*r + u3*fp.l/fp.Iyy;
rdot     = (fp.Ixx-fp.Iyy)/fp.Izz*p*q + u4/fp.Izz;

dposition     = [xdot;ydot;zdot];
dvelocity     = [udot;vdot;wdot];
deuler_angles = [psidot;thetadot;phidot];
drates        = [pdot;qdot;rdot];

dx = [dposition;deuler_angles;dvelocity;drates];

end

function Z = obs(X)

x     = X(1);
y     = X(2);
z     = X(3);
psi   = X(4);
theta = X(5);
phi   = X(6);
u     = X(7);
v     = X(8);
w     = X(9);
p     = X(10);
q     = X(11);
r     = X(12);

Z = [x;y;z;psi;theta;phi;p;q;r];

end