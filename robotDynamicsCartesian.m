function [Mx,Cx,Gx] = robotDynamicsCartesian(q, qd)

% Analytical Jacobian
J = robotJacobian(q);

% Jacobians
Jinv = pinv(J);             % Inverse
Jtrans = J';                % Transpose
JtransInv = pinv(Jtrans);   % Transpose inverse

% Jacobian dot
Jdot = robotJacobianDot(q, qd);
      
% Joint space dynamics
[Mq,Cq,Gq] = robotDynamics(q, qd);

% Cartesian space dynamics
Mx = JtransInv*Mq*Jinv;
Cx = JtransInv*( Cq - Mq*Jinv*Jdot )*Jinv;
Gx = JtransInv*Gq;

end