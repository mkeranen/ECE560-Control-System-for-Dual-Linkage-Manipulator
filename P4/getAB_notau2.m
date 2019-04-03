function [A B] = getAB_notau2(x_d,params)
    
%This function finds the linearized A and B matrices for our nonlinear
%equation, assuming also that tau2 is 0 at all times.

    theta1=x_d(1);
    dtheta1=x_d(2);
    theta2=x_d(3);
    dtheta2=x_d(4);
    
    m1 = params(1);
    m2 = params(2);
    l1 = params(3);
    l2 = params(4);
    g = params(5);
    
    a1=m2*l2^2;
    a2=m2*l1*l2;
    a3=m1*l1^2;
    a4=(m1+m2)*l1;
    a5=m2*l2;
    
    syms x1 x2 x3 x4 u1 u2
    
    M=[a1+2*a2*cos(x3)+a3 a1+a2*cos(x3) ; a1+a2*cos(x3) a1];
    V=[-a2*sin(x3)*(x4^2+2*x2*x4); a2*sin(x3)*x2^2];
    G=[a4*g*cos(x1)+a5*g*cos(x1+x3) ; a5*g*cos(x1+x3)];
    C=-inv(M)*V-inv(M)*G+inv(M)*[u1;0];
    
    
    A=jacobian([x2,C(1),x4,C(2)],[x1,x2,x3,x4]);
    B=jacobian([x2,C(1),x4,C(2)],u1);
    
    A=subs(A,x1,theta1);
    A=subs(A,x2,dtheta1);
    A=subs(A,x3,theta2);
    A=subs(A,x4,dtheta2);
    A=subs(A,u1,x_d(1));
    A=double(A);
    
    B=subs(B,x1,theta1);
    B=subs(B,x2,dtheta1);
    B=subs(B,x3,theta2);
    B=subs(B,x4,dtheta2);
    B=subs(B,u1,x_d(1));
    B=double(B);
end