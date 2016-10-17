function x = robotForwardKinematics(q)
global length

% Arm parameters
a1 = length(1);
a2 = length(2);

% Analytical Jacobian
x = [ a1*cos(q(1)) + a2*cos(q(1) + q(2))  ;
      a1*sin(q(1)) + a2*sin(q(1) + q(2)) ];

end