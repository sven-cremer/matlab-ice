function [Mq,Cq,Gq] = robotDynamics(q, qd)
global rMass rLength gravity

% Arm parameters
m1 = rMass(1);       % Center of mass is located at a1, i.e. ac1=a1
m2 = rMass(2);
a1 = rLength(1);
a2 = rLength(2);
g  = gravity;

% Input variables
q1 = q(1);
q2 = q(2);
dq1 = qd(1);
dq2 = qd(2);
    
% Inertia term M(q)
M11 = (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(q2) ;
M12 = m2*a2^2 + m2*a1*a2*cos(q2) ;
M22 = m2*a2^2 ;
Mq = [M11, M12; M12, M22];

% Coriolis term C(q,dq)
h = -m2*a1*a2*sin(q2);
C11 = h*dq2 ;
C12 = h*(dq1+dq2) ;
C21 = -h*dq1;
C22 = 0;
Cq = [C11, C12; C21, C22];

% Gravity term G(q)
G1 = (m1 + m2)*g*a1*cos(q1) + m2*g*a2*cos(q1+q2);
G2 = m2*g*a2*cos(q1 + q2);
Gq = [G1; G2];

end