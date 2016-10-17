function J = robotJacobian(q)
global length

% Arm parameters
a1 = length(1);
a2 = length(2);

% Analytical Jacobian
J = [ - sin(q(1))*a1 - a2*sin(q(1) + q(2)), - a2*sin(q(1) + q(2))  ;
    cos(q(1))*a1 + a2*cos(q(1) + q(2)),   a2*cos(q(1) + q(2)) ];

end