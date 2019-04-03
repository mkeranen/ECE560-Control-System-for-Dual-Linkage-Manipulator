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
x_d = [pi/2;0;0;0]; %Desired state and linearization point

tspan = 0:0.01:8;

% PROBLEM 2
poles = [-2.1 -2.2 -2.5 -2.6;       % Poles near the origin
         -5 -6 -7 -8;               % Poles far from the origin
         -1-2i -1+2i -1-4i -1+4i];  % Complex poles

for n = 1:size(poles,1)
    
    
    % Separate helper function to calculate K using linearized system via
    % Jacobian linearization
    K = calc_K(x_d,params,poles(n,:));
    
    % Perturbation from desired state
    init = [pi/2+0.1;0.1;-0.1;0.1];  
    
    % Simulate performance using nonlinear system and calculated K matrix 
    % from linear system
    [t, x] = ode45(@(t,x,u)ManipulatorNonlinearSystem(t, x, -K*(x-x_d), x_d, K, params), tspan, init);
    
    theta1(:,n) = real(x(:,1));
    dtheta1(:,n) = real(x(:,2));
    theta2(:,n) = real(x(:,3));
    dtheta2(:,n) = real(x(:,4));
        
    % Input Torque
    for j=1:length(x)
        inputTorque(j,n) = -K*(x(j)-x_d);
        
    end
end
%% plots

figure(1)
subplot(2,1,1)
plot(t,theta1)
title('\theta_1 State Trajectory for selected pole locations')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');
legend('Near Origin','Far from Origin', 'Complex');
subplot(2,1,2)
plot(t,theta2)
title('\theta_2 State Trajectory for selected pole locations')
xlabel('Time (seconds)'); ylabel('Angle (Radians)');
legend('Near Origin','Far from Origin', 'Complex');
figure(2)
plot(t,inputTorque);
title('Input torque requirements for different pole locations');
xlabel('Time (seconds)'); ylabel('Torque (N*m)');
legend('Near Origin','Far from Origin', 'Complex');

%% Animation             
% % Uncomment the following lines for video.
% simulationFrameRate = 100;
% animation = ManipulatorDraw('Manipulator', simulationFrameRate);
% %animation.EnablePlotRecoder();            
% animation.Draw(theta1, theta2, t);
% animation.Close();
% 











        