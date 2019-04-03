function dx = ManipulatorNonlinearSystem(t, x, u, x_d, K, params)
    
    m1 = params(1);
    m2 = params(2);
    l1 = params(3);
    l2 = params(4);
    g = params(5);
    
    theta1 = x(1);
    dtheta1 = x(2);
    theta2 = x(3);
    dtheta2 = x(4);

    % -----------------------------
    % PUT YOUR CODE HERE
    
    M = [m2*l2^2+2*m2*l1*l2*cos(theta2)+m1*l1^2 m2*l2^2+m2*l1*l2*cos(theta2); m2*l1*l2*cos(theta2)+m2*l2^2 m2*l2^2];
    V = [-1*m2*l1*l2*sin(theta2)*(dtheta2^2+2*dtheta1*dtheta2) ; m2*l1*l2*sin(theta2)*dtheta1^2];
    G = [(m1+m2)*l1*g*cos(theta1)+m2*l2*g*cos(theta1+theta2) ; m2*l2*g*cos(theta1+theta2)];
    
    
    % -----------------------------
    
    ddTheta = -inv(M)*V-inv(M)*G+inv(M)*[u;0];  % tau2 = 0
    
    dx = [dtheta1; ddTheta(1); dtheta2; ddTheta(2)]
    
end
