function [xdot,F] = compute_xdot_and_F(x,gyro_wb,accel_fb)

g = 9.81;

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

C_bodyrate2eulerdot  = [1      sin(phi)*tan(theta)    cos(phi)*tan(theta); ...
                        0           cos(phi)                -sin(phi)    ; ...
                        0      sin(phi)*sec(theta)    cos(phi)*sec(theta)];

C_ned2b  = [cos(theta)*cos(psi)                               cos(theta)*sin(psi)                             -sin(theta); ...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)   sin(phi)*cos(theta); ...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)   cos(phi)*cos(theta)];
C_b2ned=transpose(C_ned2b);

xdot = [ ...
    C_bodyrate2eulerdot*([wx;wy;wz]-[bwx;bwy;bwz]); ...  % Derivative of [roll; pitch; yaw]
    [Vn; Ve; -Vd]; ...                                   % Derivative of [Pn; Pe; Alt]
    C_b2ned*([fx;fy;fz]-[bax;bay;baz])+[0;0;g]; ... % Derivative of [Vn; Ve; Vd]
    [0;0;0]; ...                                         % Derivative of [bwx; bwy; bwz]
    [0;0;0]; ...                                         % Derivative of [bax; bay; baz]
    ];

F = [ ...
    [                                                                 sin(phi)*tan(theta)*(bwz - wz) - cos(phi)*tan(theta)*(bwy - wy),                                  - cos(phi)*(bwz - wz)*(tan(theta)^2 + 1) - sin(phi)*(bwy - wy)*(tan(theta)^2 + 1),                                                                                                                                                              0, 0, 0, 0, 0, 0,  0, -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta),                    0,                                                  0,                                                  0]
    [                                                                                       cos(phi)*(bwz - wz) + sin(phi)*(bwy - wy),                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,            -cos(phi),             sin(phi),                    0,                                                  0,                                                  0]
    [                                                             (sin(phi)*(bwz - wz))/cos(theta) - (cos(phi)*(bwy - wy))/cos(theta),                    - (cos(phi)*sin(theta)*(bwz - wz))/cos(theta)^2 - (sin(phi)*sin(theta)*(bwy - wy))/cos(theta)^2,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0, -sin(phi)/cos(theta), -cos(phi)/cos(theta),                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 1, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 1,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0, -1,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [ - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(bay - fy) - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(baz - fz), cos(psi)*sin(theta)*(bax - fx) - cos(phi)*cos(psi)*cos(theta)*(baz - fz) - cos(psi)*cos(theta)*sin(phi)*(bay - fy), (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(bay - fy) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(baz - fz) + cos(theta)*sin(psi)*(bax - fx), 0, 0, 0, 0, 0,  0,  0,                    0,                    0, -cos(psi)*cos(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta), - sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)]
    [   (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(bay - fy) + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(baz - fz), sin(psi)*sin(theta)*(bax - fx) - cos(phi)*cos(theta)*sin(psi)*(baz - fz) - cos(theta)*sin(phi)*sin(psi)*(bay - fy), (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(bay - fy) - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(baz - fz) - cos(psi)*cos(theta)*(bax - fx), 0, 0, 0, 0, 0,  0,  0,                    0,                    0, -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta),   cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)]
    [                                                                 cos(theta)*sin(phi)*(baz - fz) - cos(phi)*cos(theta)*(bay - fy),                            cos(theta)*(bax - fx) + cos(phi)*sin(theta)*(baz - fz) + sin(phi)*sin(theta)*(bay - fy),                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,           sin(theta),                               -cos(theta)*sin(phi),                               -cos(phi)*cos(theta)]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    [                                                                                                                               0,                                                                                                                  0,                                                                                                                                                              0, 0, 0, 0, 0, 0,  0,  0,                    0,                    0,                    0,                                                  0,                                                  0]
    ];