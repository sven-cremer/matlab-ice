function Jdot = robotJacobianDot(q,qd)
global rLength

% Arm parameters
a1 = rLength(1);
a2 = rLength(2);

% Jacobian dot
Jdot = [ - cos(q(1))*a1*qd(1) - a2*cos(q(1) + q(2))*(qd(1) + qd(2)), ...
    - a2*cos(q(1) + q(2))*(qd(1) + qd(2))  ;
    
    -sin(q(1))*a1*qd(1) - a2*sin(q(1) + q(2))*(qd(1) + qd(2)) , ...
    -a2*sin(q(1) + q(2))*(qd(1) + qd(2)) ];

end