%% Navigation Example (for IBCAST 2022 Paper # 04)

clear;close all;clc
format long g
warning off
set(0,'DefaultLineLineWidth',2);

rng(8); % For Reproducibility

%% Load Truth Motion Data

g = 9.81; % m/s^2

load UavData_SquareSpiral
% load UavData_Loiter

true_time    = uavTruth.time_s;             %s
true_roll    = uavTruth.roll_deg*pi/180;    %rad
true_pitch   = uavTruth.pitch_deg*pi/180;   %rad
true_yaw     = uavTruth.yaw_deg*pi/180;     %rad
true_Pnorth  = uavTruth.north_m;            %m
true_Peast   = uavTruth.east_m;             %m
true_Pheight = uavTruth.h_msl_m;            %m
true_Vnorth  = uavTruth.v_ned_mps(:,1);     %m/s
true_Veast   = uavTruth.v_ned_mps(:,2);     %m/s
true_Vdown   = uavTruth.v_ned_mps(:,3);     %m/s
true_wbx     = uavTruth.wb_rps(:,1);        %rad/s
true_wby     = uavTruth.wb_rps(:,2);        %rad/s
true_wbz     = uavTruth.wb_rps(:,3);        %rad/s

true_Vdot_north = gradient(true_Vnorth,true_time);
true_Vdot_east  = gradient(true_Veast,true_time);
true_Vdot_down  = gradient(true_Vdown,true_time);

true_Vdot_ned = [true_Vdot_north true_Vdot_east true_Vdot_down];

N = length(true_time);

true_C_ned2b = zeros(3,3,N);
true_fb    = zeros(N,3);

for i = 1:N
    true_C_ned2b(:,:,i) = rotx(true_roll(i))*roty(true_pitch(i))*rotz(true_yaw(i));
    true_fb(i,:) = true_C_ned2b(:,:,i)*true_Vdot_ned(i,:).' - true_C_ned2b(:,:,i)*[0;0;g];
end

true_fbx = true_fb(:,1); %m/s^2
true_fby = true_fb(:,2); %m/s^2
true_fbz = true_fb(:,3); %m/s^2

%% Noises & Biases

% Sensor Noises & Biases (Sigmas)
gyro_noise = 0.0017; %rad/s
gyro_bias  = 0.01;   %rad/s

accel_noise = 0.01;  %m/s^2
accel_bias  = 0.1;   %m/s^2

magnetometer_noise = 0.02;
magnetometer_bias  = 0.0;

GPS_pos_noise = 1; %m
GPS_pos_bias  = 0; %m

GPS_vel_noise = 0.05; %m/s
GPS_vel_bias  = 0;    %m/s

% Measurement Noises (Sigmas)
yaw_measurement_noise = sqrt(magnetometer_noise^2 + 5*magnetometer_bias^2);
pos_measurement_noise = sqrt(GPS_pos_noise^2 + 5*GPS_pos_bias^2);
vel_measurement_noise = sqrt(GPS_vel_noise^2 + 5*GPS_vel_bias^2);

% Process Noises (Sigmas)
attitude_process_noise  = 0.002; %rad
pos_process_noise       = 0; %m
vel_process_noise       = 2; %m/s
gyroBias_process_noise  = 1e-6; %rad/s
accelBias_process_noise = 1e-6; %rad/s

%% Generate Sensor Data
% GPS

dt_GPS = 0.1; % GPS Update Rate

GPS_Pnorth  = true_Pnorth  + GPS_pos_bias*randn + GPS_pos_noise*randn(size(true_time));
GPS_Peast   = true_Peast   + GPS_pos_bias*randn + GPS_pos_noise*randn(size(true_time));
GPS_Pheight = true_Pheight + GPS_pos_bias*randn + GPS_pos_noise*randn(size(true_time));

GPS_Vnorth = true_Vnorth + GPS_vel_bias*randn + GPS_vel_noise*randn(size(true_time));
GPS_Veast  = true_Veast  + GPS_vel_bias*randn + GPS_vel_noise*randn(size(true_time));
GPS_Vdown  = true_Vdown  + GPS_vel_bias*randn + GPS_vel_noise*randn(size(true_time));

kGPS = interp1(true_time,1:length(true_time),true_time(1):dt_GPS:true_time(end),'nearest');
GPS_valid_flag = false(size(true_time));
GPS_valid_flag(kGPS) = true;

GPS_Pnorth(~GPS_valid_flag)  = NaN;
GPS_Peast(~GPS_valid_flag)   = NaN;
GPS_Pheight(~GPS_valid_flag) = NaN;
GPS_Vnorth(~GPS_valid_flag)  = NaN;
GPS_Veast(~GPS_valid_flag)   = NaN;
GPS_Vdown(~GPS_valid_flag)   = NaN;

%% IMU Data

gyro_biases = gyro_bias*rand(1,3);

gyro_wbx = true_wbx + gyro_biases(1) + gyro_noise*randn(size(true_time));
gyro_wby = true_wby + gyro_biases(2) + gyro_noise*randn(size(true_time));
gyro_wbz = true_wbz + gyro_biases(3) + gyro_noise*randn(size(true_time));


accel_biases = accel_bias*rand(1,3);

accel_fbx = true_fbx + accel_biases(1) + accel_noise*randn(size(true_time));
accel_fby = true_fby + accel_biases(2) + accel_noise*randn(size(true_time));
accel_fbz = true_fbz + accel_biases(3) + accel_noise*randn(size(true_time));

%% Magnetometer Data

mag_declination = 10*randn*pi/180;

C_mag2ned = rotz(-mag_declination);

magnetometer_bias = magnetometer_bias*randn(1,3);

magnetometer_yaw = zeros(size(true_time));
for i = 1:N
    M = true_C_ned2b(:,:,i)*C_mag2ned*[1;0;0] + magnetometer_bias' + ...
        magnetometer_noise*randn(1,3)';
    
    M = M/norm(M);
    
    magnetometer_yaw(i) = atan2(-M(2),M(1)) + mag_declination;
    magnetometer_yaw(i) = mod(magnetometer_yaw(i)+pi,2*pi)-pi; % [-pi,+pi]
    
end

%% State Estimation with Extended Kalman Filter

Aided_Navigation_Flag = 1;

% x = [phi,theta,psi,Pn,Pe,Alt,Vn,Ve,Vd,bwx,bwy,bwz,bax,bay,baz]

% Initialize States
x0 = [0;
    0;
    magnetometer_yaw(1);
    GPS_Pnorth(1);
    GPS_Peast(1);
    GPS_Pheight(1);
    GPS_Vnorth(1);
    GPS_Veast(1);
    GPS_Vdown(1);
    0;
    0;
    0;
    0;
    0;
    0];

xhat = zeros(15,length(true_time));
xhat(:,1) = x0;

nStates = length(x0);

% Initialize Covariance Matrix
large_angle_uncertainty = 30*pi/180;
large_pos_uncertainty   = 100;
large_vel_uncertainty   = 10;

P = diag([...
    [1 1 1]*large_angle_uncertainty ... % init Euler angle (NED-to-body) uncertainties, rad
    [1 1 1]*large_pos_uncertainty ...   % init North-East-Alt position uncertainties, m
    [1 1 1]*large_vel_uncertainty ...   % init NED velocity uncertainties, m/s
    [1 1 1]*gyro_bias ...               % init XYZ gyro bias uncertainties, rad/s
    [1 1 1]*accel_bias ...              % init XYZ accel bias uncertainties, m/s^2
    ].^2);

P = max(P,(1e-3)^2*eye(size(P)));

P_est(1,:) = diag(P);

% Initialize Process Noise Covariance Matrix

Q = diag([...
    [1 1 1]*attitude_process_noise...   % Euler angle process noise, rad
    [1 1 1]*pos_process_noise ...       % North-East-Alt position process noise, m
    [1 1 1]*vel_process_noise ...       % NED velocity process noise, m/s
    [1 1 1]*gyroBias_process_noise ...  % XYZ gyro bias process noise, rad/s
    [1 1 1]*accelBias_process_noise ... % XYZ accel bias process noise, m/s^2
    ].^2);

Q = max(Q,(1e-3)^2*eye(size(Q)));

% Initialize Measurement Noise Covariance Matrix
SF_magUnc = 1;
R = diag([...
    [1]*SF_magUnc*yaw_measurement_noise ... % Magnetometer-based yaw measurement noise, rad
    [1 1 1]*pos_measurement_noise ...   % North-East-Alt position measurement noise, m
    [1 1 1]*vel_measurement_noise ...   % NED velocity measurement noise, m/s
    ].^2);
R = max(R,(1e-3)^2*eye(size(R)));

% [P,Q,R] = deal(P*500,Q*500,R*500);

% Estimation Loop

for k = 2:N
    
    dt_s = true_time(k) - true_time(k-1);
    
    if GPS_valid_flag
        
        yaw_measurement = magnetometer_yaw(k);
        
        while yaw_measurement>xhat(3,k-1)+pi, yaw_measurement=yaw_measurement-2*pi; end
        while yaw_measurement<xhat(3,k-1)-pi, yaw_measurement=yaw_measurement+2*pi; end
        
        
        z = [yaw_measurement;
            GPS_Pnorth(k);
            GPS_Peast(k);
            GPS_Pheight(k);
            GPS_Vnorth(k);
            GPS_Veast(k);
            GPS_Vdown(k)];
        
        if k>1 && (abs(xhat(1,k-1))>5*pi/180 || abs(gyro_wbx(k))>3*pi/180)
            SF_magUnc=10; % Magnetometer uncertainty Scale Factor
        else
            SF_magUnc=1;  % Magnetometer uncertainty Scale Factor
        end
        
        R = diag([...
            [1]*SF_magUnc*yaw_measurement_noise ... % Magnetometer-based yaw measurement noise, rad
            [1 1 1]*pos_measurement_noise ...   % North-East-Alt position measurement noise, m
            [1 1 1]*vel_measurement_noise ...   % NED velocity measurement noise, m/s
            ].^2);
        R = max(R,(1e-3)^2*eye(size(R)));
        
    else
        
        z = [];
        R = [];
        
    end
    
    gyro_wb  = [gyro_wbx(k);gyro_wby(k);gyro_wbz(k)];
    accel_fb = [accel_fbx(k);accel_fby(k);accel_fbz(k)];
    
    [dx, F] = compute_xdot_and_F(xhat(:,k-1),gyro_wb,accel_fb);
    
    % Estimation (Pure INS)
    xhat(:,k) = xhat(:,k-1) + dx(:)*dt_s;
    
    % GPS-Correction with EKF
    if (Aided_Navigation_Flag)
        
        AA = [-F Q; zeros(nStates,nStates) F']*dt_s;
        BB = expm(AA);
        PHI_k = BB(nStates+1:2*nStates,nStates+1:2*nStates)';
        Q_k = PHI_k*BB(1:nStates,nStates+1:2*nStates);
        
        P = PHI_k*P*PHI_k' + Q_k;
        
        if ~isempty(z)
            [zhat, H] = compute_zhat_and_H(xhat(:,k));
            K = (P*H')/(H*P*H'+R);
            xhat(:,k) = xhat(:,k) + K*(z-zhat);
            P = (eye(length(xhat(:,k)))-K*H)*P;
        end
        
    end
    
    P_est(k,:) = diag(P);
    
end

estimated_roll  = xhat(1,:)*180/pi;
estimated_pitch = xhat(2,:)*180/pi;
estimated_yaw   = xhat(3,:)*180/pi;

estimated_Pnorth  = xhat(4,:);
estimated_Peast   = xhat(5,:);
estimated_Pheight = xhat(6,:);

estimated_Vnorth = xhat(7,:);
estimated_Veast  = xhat(8,:);
estimated_Vdown  = xhat(9,:);

%%

figure(1);hold on;grid on;box on
plot(GPS_Peast,GPS_Pnorth,'gd','MarkerSize',6);
plot(estimated_Peast,estimated_Pnorth,'r.-');
plot(true_Peast,true_Pnorth,'b.-');
xlabel('East, m');ylabel('North, m');
title('UAV Position');
legend('GPS Measurement','Estimated Position','True Position');

figure(2);hold on;grid on;box on
plot(true_time,GPS_Pheight,'gd','MarkerSize',6);
plot(true_time,estimated_Pheight,'r.-');
plot(true_time,true_Pheight,'b.-');
xlabel('Time, s');ylabel('Altitude, m');
title('UAV Altitude');
legend('GPS Altitude','Estimated Altitude','True Altitude');

figure(3);hold on;grid on;box on
mag = @(v) (sqrt(sum(v.^2,2)));
plot(true_time,mag([GPS_Vnorth(:),GPS_Veast(:),GPS_Vdown(:)]),'gd','MarkerSize',6);
plot(true_time,mag([estimated_Vnorth(:),estimated_Veast(:),estimated_Vdown(:)]),'r.-');
plot(true_time,mag([true_Vnorth(:),true_Veast(:),true_Vdown(:)]),'b.-');
xlabel('Time, s');ylabel('Inertial Speed, m/s');
title('UAV Speed');
legend('GPS Speed','Estimated Speed','True Speed');

figure(4);hold on;grid on;box on
plot(true_time,estimated_roll,'r.-');
plot(true_time,true_roll*180/pi,'b.-');
xlabel('Time, s');ylabel('Roll Angle, deg');
title('UAV Roll');
legend('Estimated Roll','True Roll');

figure(5);hold on;grid on;box on
plot(true_time,estimated_pitch,'r.-');
plot(true_time,true_pitch*180/pi,'b.-');
xlabel('Time, s');ylabel('Pitch Angle, deg');
title('UAV Pitch');
legend('Estimated Pitch','True Pitch');

figure(6);hold on;grid on;box on
wrap = @(angle) (mod(angle+180,360)-180);
plot(true_time,wrap(magnetometer_yaw*180/pi),'r.-');
plot(true_time,wrap(estimated_yaw),'r.-');
plot(true_time,wrap(true_yaw*180/pi),'b.-');
xlabel('Time, s');ylabel('Yaw Angle, deg');
title('UAV Yaw');
legend('Magnetometer Yaw','Estimated Yaw','True Yaw');

figure(7);clf

estimates     = 180/pi*xhat(1:3,:).';
uncertainties = 180/pi*sqrt(P_est(:,1:3));
truths        = 180/pi.*[true_roll true_pitch true_yaw];

subplot(311);hold on;grid on;box on
plot(true_time,uncertainties(:,1)*[-1,1],'b.-');
plot(true_time,truths(:,1)-estimates(:,1),'r.-');
ylabel('Roll Error [deg]');

subplot(312);hold on;grid on;box on
plot(true_time,uncertainties(:,2)*[-1,1],'b.-');
plot(true_time,truths(:,2)-estimates(:,2),'r.-');
ylabel('Pitch Error [deg]');

subplot(313);hold on;grid on;box on
plot(true_time,uncertainties(:,3)*[-1,1],'b.-');
plot(true_time,truths(:,3)-estimates(:,3),'r.-');
ylabel('Yaw Error [deg]');
xlabel('Time [sec]');

figure(8);clf

estimates     = xhat(4:6,:).';
uncertainties = sqrt(P_est(:,4:6));
truths        = [true_Pnorth true_Peast true_Pheight];

subplot(311);hold on;grid on;box on
plot(true_time,uncertainties(:,1)*[-1,1],'b.-');
plot(true_time,truths(:,1)-estimates(:,1),'r.-');
ylabel('North Position Error [m]');

subplot(312);hold on;grid on;box on
plot(true_time,uncertainties(:,2)*[-1,1],'b.-');
plot(true_time,truths(:,2)-estimates(:,2),'r.-');
ylabel('East Position Error [m]');

subplot(313);hold on;grid on;box on
plot(true_time,uncertainties(:,3)*[-1,1],'b.-');
plot(true_time,truths(:,3)-estimates(:,3),'r.-');
ylabel('Altitude Error [deg]');
xlabel('Time [sec]');

figure(9);clf

estimates     = xhat(7:9,:).';
uncertainties = sqrt(P_est(:,7:9));
truths        = [true_Vnorth true_Veast true_Vdown];

subplot(311);hold on;grid on;box on
plot(true_time,uncertainties(:,1)*[-1,1],'b.-');
plot(true_time,truths(:,1)-estimates(:,1),'r.-');
ylabel('North Velocity Error [m]');

subplot(312);hold on;grid on;box on
plot(true_time,uncertainties(:,2)*[-1,1],'b.-');
plot(true_time,truths(:,2)-estimates(:,2),'r.-');
ylabel('East Velocity Error [m]');

subplot(313);hold on;grid on;box on
plot(true_time,uncertainties(:,3)*[-1,1],'b.-');
plot(true_time,truths(:,3)-estimates(:,3),'r.-');
ylabel('Down Velocity Error [deg]');
xlabel('Time [sec]');

figure(10);clf

estimates     = xhat(10:12,:).';
uncertainties = sqrt(P_est(:,10:12));
truths        = ones(size(true_time))*gyro_biases;

subplot(311);hold on;grid on;box on
plot(true_time,uncertainties(:,1)*[-1,1],'b.-');
plot(true_time,truths(:,1)-estimates(:,1),'r.-');
ylabel('x-Gyro Bias [m]');

subplot(312);hold on;grid on;box on
plot(true_time,uncertainties(:,2)*[-1,1],'b.-');
plot(true_time,truths(:,2)-estimates(:,2),'r.-');
ylabel('y-Gyro Bias [m]');

subplot(313);hold on;grid on;box on
plot(true_time,uncertainties(:,3)*[-1,1],'b.-');
plot(true_time,truths(:,3)-estimates(:,3),'r.-');
ylabel('z-Gyro Bias [deg]');
xlabel('Time [sec]');

figure(11);clf

estimates     = xhat(13:15,:).';
uncertainties = sqrt(P_est(:,13:15));
truths        = ones(size(true_time))*accel_biases;

subplot(311);hold on;grid on;box on
plot(true_time,uncertainties(:,1)*[-1,1],'b.-');
plot(true_time,truths(:,1)-estimates(:,1),'r.-');
ylabel('x-Accel Bias [m]');

subplot(312);hold on;grid on;box on
plot(true_time,uncertainties(:,2)*[-1,1],'b.-');
plot(true_time,truths(:,2)-estimates(:,2),'r.-');
ylabel('y-Accel Bias [m]');

subplot(313);hold on;grid on;box on
plot(true_time,uncertainties(:,3)*[-1,1],'b.-');
plot(true_time,truths(:,3)-estimates(:,3),'r.-');
ylabel('z-Accel Bias [deg]');
xlabel('Time [sec]');