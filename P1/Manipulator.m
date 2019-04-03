close all;

m1 = 2;
m2 = 1;
l1 = 1;
l2 = 1;
g = 9.81;
params = [m1, m2, l1, l2, g];

%% Run Simulation

u = [0; 0];
x_d = 0;
K = 0;

% In this case, init is perturbation
init = [0.1; 0; 0; 0];  
tspan = 0:0.1:10;

%[t, x] = ode45(@(t,x)ManipulatorNonlinearSystem(t, x, u, x_d, K, params), tspan, init);
[t, x] = ode45(@(t,x)ManipulatorLinearSystem(t, x, u, x_d, K, params), tspan, init);

% Correct the orientation from linearization
theta1 = x(:,1)-(pi/2);
dtheta1 = x(:,2);
theta2 = x(:,3);
dtheta2 = x(:,4);

%% plots

%doing the plot for theta_1 and theta_2
figure(1)
subplot(2,1,1)
plot(t,theta1)
title('\theta_1 State Trajectory')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');
subplot(2,1,2)
plot(t,theta2)
title('\theta_2 State Trajectory')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');

        
    
%% Animation             
% Uncomment the following lines for video.
simulationFrameRate = 10;
animation = ManipulatorDraw('Manipulator', simulationFrameRate);
%animation.EnablePlotRecoder();            
animation.Draw(theta1, theta2, t);
animation.Close();












        