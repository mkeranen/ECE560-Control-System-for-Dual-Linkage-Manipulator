close all;
clear all;

m1 = 2;
m2 = 1;
l1 = 1;
l2 = 1;
g = 9.81;
params = [m1, m2, l1, l2, g];

%% Run Simulation

u=0; %redefining our input u to be just tau1
x_d = [pi/3;0;-5*pi/6;0]; % Desired state and linearization point


% PROBLEM 4
%finding the K using LQR
%know that A is 4x4, so Q must be 4x4
%know that u is just tau1, so R is just one value
%our Q is optimized such that theta1 and theta2 are more important, meaning
%we weigh them higher (we want the error to be smaller).
Q=[1000 0 0 0;0 1 0 0;0 0 1000 0;0 0 0 1]; 
R=0.1; %tau can be a bit larger to ensure stable angles and velocities
[A, B] = getAB_notau2(x_d,params);
K=lqr(A,B,Q,R);


tspan = 0:0.05:8;

% Add perturbation to initial state if desired
init = [pi/3-.1;0;0;0];

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

%% plots

%doing the plot for theta_1 and theta_2
figure(1)
subplot(2,1,1)
plot(t,theta1)
title('\theta_1 State Trajectory for LQR optimized poles; Target \pi/3')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');
subplot(2,1,2)
plot(t,theta2)
title('\theta_2 State Trajectory for LQR optimized poles')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');

        
    
%% Animation             
% Uncomment the following lines for video.
simulationFrameRate = 100;
animation = ManipulatorDraw('Manipulator', simulationFrameRate);
%animation.EnablePlotRecoder();            
animation.Draw(theta1, theta2, t);
animation.Close();












        