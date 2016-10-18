% Authors: Isura Ranatunga
%          Sven Cremer
% University of Texas at Arlington
% Dept. of Electrical Engineering
% UT Arlington Research Institute
%
% Email address: isura@ieee.org
%                sven.cremer@mavs.uta.edu
%
% Website: isura.me
%          svencremer.com
%
% Created:  08/16/2013 - IR
% Modified: 04/28/2014 - IR
%           10/17/2016 - SC
%
% Cartesian Space Model of a planar 2-link RR arm to be called by MATLAB
% function ode23 or ode45

%------------- BEGIN CODE --------------

function xdot = robotModel( t, x )

% --------------------------------------
% ROBOT STATE
global tau

q  = [x(1) x(2)]';
qd = [x(3) x(4)]';

% --------------------------------------
% ROBOT ARM DYNAMICS SIMULATION

[Mq,Cq,Gq] = robotDynamics(q, qd);

% Arm dynamics
qdd = inv(Mq)*(-Cq*qd-Gq+tau);
         
% Disturbance turque tau_d
%         taud = 1;
%         tau1 = tau1 + taud;
%         tau1 = tau1 + taud;
       
% Human disturbance
%         tau_hum = Jtrans*f_hum;

% --------------------------------------

% Update state equation
xdot= [ qd(1)  ;
        qd(2)  ;
        qdd(1) ;
        qdd(2) ];

end
%-------------- END CODE ---------------
