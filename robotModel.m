% Author: Isura Ranatunga,
% University of Texas at Arlington
% Dept. of Electrical Engineering
% UT Arlington Research Institute
%
% email address: isura@ieee.org
% Website: isura.me
% Created: 08/16/2013
% Modified: 04/28/2014
%
% Neuroadaptive Control of planar 2 link RR arm
% using 2 layer Augmented Neural Network
% Cartesian Space Model Joint Space Input (Torque)
%

%------------- BEGIN CODE --------------

% file robout2layerJntSpModelJntSpIn.m, to be called by MATLAB function ode23
function xdot= robotModel( t, x )

global Mx Cx Gx fc_exp  % Used in neuroAdpative controller for comparision

global t_prev
if(~isempty(t_prev))
    dt = t-t_prev;
else
    dt = 0;
end

% -----------------------------------------------------
% ARM PARAMETERS
global length
a1 = length(1);
a2 = length(2);

% -----------------------------------------------------
% MODEL STATE
global x_m_ xd_m_ xdd_m_

q_m   = x_m_   ;
qd_m  = xd_m_  ;
qdd_m = xdd_m_ ;

% -----------------------------------------------------
% ROBOT STATE

% Robot states
q  = [x(1) x(2)]';
qd = [x(3) x(4)]';

% Forward kinematics
xC = robotForwardKinematics(q);

% Analytical Jacobian
J = robotJacobian(q);

xdC= J*qd;

% -----------------------------------------------------
% TODO: move -->
% ROBOT DYNAMICS

% Jacobians
Jinv = pinv(J);             % Inverse
Jtrans = J';                % Transpose
JtransInv = pinv(Jtrans);   % Transpose inverse

% Jacobian dot
Jdot = [ - cos(q(1))*a1*qd(1) - a2*cos(q(1) + q(2))*(qd(1) + qd(2)), ...
    - a2*cos(q(1) + q(2))*(qd(1) + qd(2))  ;
    
    -sin(q(1))*a1*qd(1) - a2*sin(q(1) + q(2))*(qd(1) + qd(2)) , ...
    -a2*sin(q(1) + q(2))*(qd(1) + qd(2)) ];
      
% Joint space dynamics
[Mq,Cq,Gq] = robotDynamics(x(1:2), x(3:4));

% Cartesian space dynamics
Mx = JtransInv*Mq*Jinv;
Cx = JtransInv*( Cq - Mq*Jinv*Jdot )*Jinv;
Gx = JtransInv*Gq;

% -----------------------------------------------------
% CONTROLLER

% NN controller
fc = neuroAdaptiveController(q, qd, xC, xdC, x_m_, xd_m_, xdd_m_,dt);

% Computed torques
global tau tau_exp
tau     =  Jtrans*fc;
tau_exp =  Jtrans*fc_exp;

% -----------------------------------------------------
% Torque saturation
tau_max = 35;
% [v_max, i_max] = max(abs(tau));
% if( v_max > tau_max)
%     tau = tau.*(tau_max/tau(i_max));
% end

if( abs(tau(1)) > tau_max)
    tau(1) = tau(1).*(tau_max/abs(tau(1)));
end
if( abs(tau(2)) > tau_max)
    tau(2) = tau(2).*(tau_max/abs(tau(2)));
end
% TODO: move <--
% -----------------------------------------------------
% ROBOT ARM DYNAMICS SIMULATION
% -----------------------------------------------------
    
% Arm dynamics
qdd = inv(Mq)*(-Cq*qd-Gq+tau);
         
% Disturbance turque tau_d
%         taud = 1;
%         tau1 = tau1 + taud;
%         tau1 = tau1 + taud;
       
% qdd joint acceleration
%         tau_hum = Jtrans*f_hum;
%         qdd(1) = MI11*(-N1 + tau(1)) + MI12*(-N2 + tau(2) + tau_hum(1));
%         qdd(2) = MI12*(-N1 + tau(1)) + MI22*(-N2 + tau(2) + tau_hum(2));
        
% -----------------------------------------------------

% State equations
xdot= [ qd(1)  ;
        qd(2)  ;
        qdd(1) ;
        qdd(2) ];

t_prev = t;
end
%-------------- END CODE ---------------
