function xnext = dots2(x,gyro_wb,accel_fb)

dt = 0.1;
g  = 9.81;

n = 1;

phi   = x(n); n=n+1; % Roll, Pitch, Yaw Euler angles, rad
theta = x(n); n=n+1;%
psi   = x(n); n=n+1; %
Pn    = x(n); n=n+1; % Position, North/East/Altitude, meters
Pe    = x(n); n=n+1; %
Alt   = x(n); n=n+1; %
Vn    = x(n); n=n+1; % Velocity, North/East/Down, m/s
Ve    = x(n); n=n+1; %
Vd    = x(n); n=n+1; %
bwx   = x(n); n=n+1; % Gyro biases, rad/s
bwy   = x(n); n=n+1; %
bwz   = x(n); n=n+1; %
bax   = x(n); n=n+1; % Accelerometer biases, m/s^2
bay   = x(n); n=n+1; %
baz   = x(n); n=n+1; %

% Angular rate measurement from gyros
wx = gyro_wb(1); % rad/s
wy = gyro_wb(2);
wz = gyro_wb(3);

% Specific force measurement from accelerometers
fx = accel_fb(1); % m/s^2
fy = accel_fb(2);
fz = accel_fb(3);

C_bodyrate2eulerdot  = [1      sin(phi).*tan(theta)    cos(phi).*tan(theta); ...
    0           cos(phi)                -sin(phi)    ; ...
    0      sin(phi).*sec(theta)    cos(phi).*sec(theta)];

C_ned2b  = [cos(theta).*cos(psi)                               cos(theta).*sin(psi)                             -sin(theta); ...
    sin(phi).*sin(theta).*cos(psi)-cos(phi).*sin(psi)    sin(phi).*sin(theta).*sin(psi)+cos(phi).*cos(psi)   sin(phi).*cos(theta); ...
    cos(phi).*sin(theta).*cos(psi)+sin(phi).*sin(psi)    cos(phi).*sin(theta).*sin(psi)-sin(phi).*cos(psi)   cos(phi).*cos(theta)];
C_b2ned=transpose(C_ned2b);

% xdot = [ ...
%     C_bodyrate2eulerdot*(); ...  % Derivative of [roll; pitch; yaw]
%     [Vn; Ve; -Vd]; ...                                   % Derivative of [Pn; Pe; Alt]
%     C_b2ned*([fx;fy;fz]-[bax;bay;baz])+[0;0;g]; ... % Derivative of [Vn; Ve; Vd]
%     [0;0;0]; ...                                         % Derivative of [bwx; bwy; bwz]
%     [0;0;0]; ...                                         % Derivative of [bax; bay; baz]
%     ];

temp1 = [wx;wy;wz]-[bwx;bwy;bwz];

xdot(1,:) = C_bodyrate2eulerdot(1,1).*temp1(1,:) + ...
    C_bodyrate2eulerdot(1,2).*temp1(2,:) + ...
    C_bodyrate2eulerdot(1,3).*temp1(3,:);

xdot(2,:) = C_bodyrate2eulerdot(2,1).*temp1(1,:) + ...
    C_bodyrate2eulerdot(2,2).*temp1(2,:) + ...
    C_bodyrate2eulerdot(2,3).*temp1(3,:);
   
xdot(3,:) = C_bodyrate2eulerdot(3,1).*temp1(1,:) + ...
    C_bodyrate2eulerdot(3,2).*temp1(2,:) + ...
    C_bodyrate2eulerdot(3,3).*temp1(3,:);

xdot(4,:) = Vn;
xdot(5,:) = Ve;
xdot(6,:) = -Vd;

temp2 = [fx;fy;fz]-[bax;bay;baz];

xdot(7,:) = C_b2ned(1,1).*temp2(1,:) + ...
    C_b2ned(1,2).*temp2(2,:) + ...
    C_b2ned(1,3).*temp2(3,:) + 0;

xdot(8,:) = C_b2ned(2,1).*temp2(1,:) + ...
    C_b2ned(2,2).*temp2(2,:) + ...
    C_b2ned(2,3).*temp2(3,:) + 0;

xdot(9,:) = C_b2ned(3,1).*temp2(1,:) + ...
    C_b2ned(3,2).*temp2(2,:) + ...
    C_b2ned(3,3).*temp2(3,:) + g;

xdot(10,:) = 0;
xdot(11,:) = 0;
xdot(12,:) = 0;
xdot(13,:) = 0;
xdot(14,:) = 0;
xdot(15,:) = 0;

xnext = x+xdot.*dt;

end