function [zhat,H] = compute_zhat_and_H(x)

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

zhat = [ ...
        [psi]; ...                    % [2D magnetometer yaw measurement]
        [Pn; Pe; Alt]; ...            % [Pn; Pe; Alt]
        [Vn; Ve; Vd]; ...             % [Vn; Ve; Vd]
       ];

H = [ ...
    [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    ];