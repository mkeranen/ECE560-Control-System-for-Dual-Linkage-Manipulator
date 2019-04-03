function K = calc_K(x_d,params,poles)
    m1 = params(1);
    m2 = params(2);
    l1 = params(3);
    l2 = params(4);
    g = params(5);
    
    theta1 = x_d(1);
    dtheta1 = x_d(2);
    theta2 = x_d(3);
    dtheta2 = x_d(4);
    
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
    
    A2=subs(A,x1,theta1);
    A2=subs(A2,x2,dtheta1);
    A2=subs(A2,x3,theta2);
    A2=subs(A2,x4,dtheta2);
    A2=double(A2);
    
    B2=subs(B,x1,theta1);
    B2=subs(B2,x2,dtheta1);
    B2=subs(B2,x3,theta2);
    B2=subs(B2,x4,dtheta2);
    B2=double(B2);
    
    K=place(A2,B2,poles);
    
end