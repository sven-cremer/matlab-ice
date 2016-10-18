function J = robotJacobian(q)
global rLength

% Arm parameters
a1 = rLength(1);
a2 = rLength(2);

% Analytical Jacobian
J = [ - sin(q(1))*a1 - a2*sin(q(1) + q(2)), - a2*sin(q(1) + q(2))  ;
    cos(q(1))*a1 + a2*cos(q(1) + q(2)),   a2*cos(q(1) + q(2)) ];

end