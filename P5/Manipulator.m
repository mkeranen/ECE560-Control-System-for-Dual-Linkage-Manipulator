close all;
clear all;
m1 = 2;
m2 = 1;
l1 = 1;
l2 = 1;
g = 9.81;
params = [m1, m2, l1, l2, g];

%% Run Simulation

u=0; 
x_d = [pi/6;0;-pi/6;0]; % Linearization point for A and B matrices

% %Problem 5 - now we have both torques to work with:
[A, B]=getAB(x_d,params);
R = [0.25 0; 0 1]; %this should ensure that tau1 max is about 4x tau2 max
Q=[50000 0 0 0;0 1 0 0;0 0 50000 0;0 0 0 1]; % prioritizing thetas over velocities
%Q needs the theta values to be large to ensure we can overcome gravity to reach the final point.
K=lqr(A,B,Q,R);

tspan = 0:0.01:2
init = [0;0;0;0];  

[t, x] = ode45(@(t,x)ManipulatorNonlinearSystem(t, x, -K*(x-x_d), x_d, K, params), tspan, init);

theta1 = real(x(:,1));
dtheta1 = real(x(:,2));
theta2 = real(x(:,3));
dtheta2 = real(x(:,4));

%right here I'm trying to extrapolate the u for every time step from x and
%x_d. we can plot this u if we want to
x_dT=transpose(x_d);
u=zeros(size(x));
for i=1:size(x)
   u(i,:)= x(i,:)-x_dT;
end

% Input Torque
for j=1:length(x)
    inputTorque(j,:) = -K*(x(j)-x_d);
    
end

%% plots

%doing the plot for theta_1 and theta_2
figure(1)
subplot(2,1,1)
plot(t,theta1)
title('\theta_1 State Trajectory for LQR optimized poles; Target \pi/6')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');
subplot(2,1,2)
plot(t,theta2)
title('\theta_2 State Trajectory for LQR optimized poles; Target -\pi/6')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');

figure(2)
plot(t,inputTorque(:,1),t,inputTorque(:,2));
title('Input torque requirements \theta_1 and \theta_2');
xlabel('Time (seconds)'); ylabel('Torque (N*m)');
legend('\theta_1','\theta_2');
        
    
%% Animation             
% Uncomment the following lines for video.
simulationFrameRate = 10;
animation = ManipulatorDraw('Manipulator', simulationFrameRate);
%animation.EnablePlotRecoder();            
animation.Draw(theta1, theta2, t);
animation.Close();












        