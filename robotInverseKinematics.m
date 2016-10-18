function q = robotInverseKinematics(x)
global rLength

q = zeros(2,1);

% Arm parameters
a1 = rLength(1);
a2 = rLength(2);

c2 = (x(1)^2 + x(2)^2 - a1^2 - a2^2)/(2*a1*a2);
s2 = sqrt(1 - c2.^2);   % plus/minus

q(2) = atan2(s2, c2);
q(1) = atan2(x(2), x(1)) - atan2(a2*s2, a1 + a2*c2);

end