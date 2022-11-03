%% Quadcopter Simulation (State Estimation with Kalman Filter)

clear;close all;clc
format long g
set(0,'DefaultLineLineWidth',2);

%%

% Initial Conditions

x0    = [0;0;0;pi/3;0;0;0;0;0;0;0;0];
zeta0 = 0.1;
xi0   = 0.1;

flagS = 1; % 1 for Actual States in Feedback, 0 for Estimated States
Tsamp = 1e-2;

%Simulation

tsim = 30;
out = sim('Quadcopter_02',[0 tsim]);

figure(1);hold on;grid on;box on
t = -0.01:0.01:30;
plot3(1/2*cos(t/2),1/2*sin(t/2),1+t/10);
plot3(out.States.data(:,1),out.States.data(:,2),out.States.data(:,3));
plot3(out.EKF.data(:,1),out.EKF.data(:,2),out.EKF.data(:,3));
view(3);legend('Reference','Tracking','Estimated');
 
EKF_States = [1,2,3,4,5,6,10,11,12];
Meas_States = 1:9;
True_States = 1:9;

for i = 1:9
    figure(i+1);hold on;grid on;box 
    h(1,i) = plot(out.Meas.time,out.Meas.data(:,Meas_States(i)),'b.-');
    h(2,i) = plot(out.EKF.time,out.EKF.data(:,EKF_States(i)),'r.-'); 
    h(3,i) = plot(out.States.time,out.States.data(:,True_States(i)),'k.-');
    legend([h(3,i),h(1,i),h(2,i)],{'True State',...
        'Measured Value','Estimated Value'},'Location','Best');
end