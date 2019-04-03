function dx = ManipulatorLinearSystem(t, x, u, x_d, K, params)
    
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
    a1=m2*l2^2;
    a2=m2*l1*l2;
    a3=m1*l1^2;
    a4=(m1+m2)*l1;
    a5=m2*l2;
    
    syms x1 x2 x3 x4 u1 u2
    
    % OKAY
    M=[a1+2*a2*cos(x3)+a3 a1+a2*cos(x3) ; a1+a2*cos(x3) a1];
    V=[-a2*sin(x3)*(x4^2+2*x2*x4); a2*sin(x3)*x2^2];
    G=[a4*g*cos(x1)+a5*g*cos(x1+x3) ; a5*g*cos(x1+x3)];
    % OKAY^
    
    C=-inv(M)*V-inv(M)*G+inv(M)*[u1;u2];
    
    
    A=jacobian([x2,C(1),x4,C(2)],[x1,x2,x3,x4]);
    B=jacobian([x2,C(1),x4,C(2)],[u1;u2]);

    
    % Plugging point into Jacobian in which to linearize about
    
    A2=subs(A,x1,0);
    A2=subs(A2,x2,0);
    A2=subs(A2,x3,0);
    A2=subs(A2,x4,0);
    A2=double(A2);
    
    B2=subs(B,x1,0);
    B2=subs(B2,x2,0);
    B2=subs(B2,x3,0);
    B2=subs(B2,x4,0);
    B2=double(B2); %we need to convert the symbolic jacobian output matrices to doubles for ode45 to work.
    
    %Here is the new calculations for G(s)
    %Since we have a MIMO system, the transfer function will be a 4x2
    %matrix
    %Still need to determine how to interpret this though.
    s=sym('s');
    I=eye(4,4);
    sI=s*I;
    C=[1 1 1 1];
    D=[0];

    %%trying it a different way here
    G=tf(ss(A2,B2,C,D));
    %%The two methods yield the same results.
    G_old=C*inv(sI-A)*B;
    % -----------------------------
    
    u=-K*(x-x_d);
    dx = (A2-B2*K)*x + B2*u;
    
    
   
end
