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
function xdot= robnn2augNeuroAdaptiveJntSpModelCartSpIn( t, x )

% -----------------------------------------------------
% ARM PARAMETERS
global length

a1 = length(1);
a2 = length(2);

%m1= 0.8 ; m2= 2.3 ;

% -----------------------------------------------------
% MODEL STATE

% Discrete model states
global q_m_z   ;
global qd_m_z  ;
global qdd_m_z ;

q_m   = q_m_z   ;
qd_m  = qd_m_z  ;
qdd_m = qdd_m_z ;

% -----------------------------------------------------
% ROBOT STATE

% Robot states
q  = [x(1) x(2)]';
qd = [x(3) x(4)]';

% Forward kinematics
xC = robotForwardKinematics(q);

% Analytical Jacobian
J = robotJacobian(q);

xCd= J*qd;

% -----------------------------------------------------
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
global input output hidden

    % NN control input
    
    % controller parameters
        Kv     = 10*eye(2)       ;      
        lam    = 20*eye(2)       ;
        F      = 100*eye(hidden) ;
        G      = 20*eye(input)   ;
        kappa  = 0.1             ;
        Kz     = 1               ;
        Zb     = 100             ;
                    
    % tracking errors
        e  = q_m  - xC   ;
        ep = qd_m - xCd  ;
        r  = ep  + lam*e ; % this is the sliding error
    
    % W and V
        nnStart = 5;
        
        endW   = (nnStart + hidden*output-1);
        startV = endW + 1;
        endV   = startV + input*hidden - 1;
        W      = reshape(x(nnStart:endW    ), output, hidden)';
        V      = reshape(x(startV:endV), input , hidden) ;
        
    % robustifying term
        Z    = [ W zeros(size(W, 1), size(V, 2)); zeros(size(V, 1), size(W, 2)) V ];
        v    = - Kz*(norm(Z) + Zb)*r;
        
    % control torques
    
        % Input to NN
        y = [ e; ep; xC; xCd; q_m; qd_m; qdd_m; q; qd ];
        
    % parameter updates
        % W update
        W_dot = F*sigmoid(V'*y)*r' - kappa*F*norm(r)*W;
        
        % V update
        sigmoidPrime = sigmoid(V'*y)+sigmoid(V'*y)'*sigmoid(V'*y);
        V_dot = G*y*(sigmoidPrime'*W*r)' - kappa*G*norm(r)*V;
        
        %V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) - ...
        %        diag(sigmoid(V'*y))))'*W*r)'  - kappa*G*norm(r)*V;

% -----------------------------------------------------
     % Computed torques

%        % PD CT control torques
%             f_true = M11*s1 + M12*s2 + N1 ;
%             f_true = M12*s1 + M22*s2 + N2 ;

        % This is the real function to be approximated
        f_rel = Mx*( qdd_m + lam*ep ) + Cx*( qd_m + lam*e ) + Gx;
%         f_hat = f_rel;
 
        % Control output
        f_hat = W'*sigmoid(V'*y);
        
         tau  =  Jtrans*( Kv*r + f_hat - v );
         %tau  =  Jtrans*f_rel;
%         tau  =  Jtrans*( Kv*r - f_hum + f_hat - v );       
        
% -----------------------------------------------------
% ROBOT ARM DYNAMICS SIMULATION
% -----------------------------------------------------
    
    % Arm dynamics
    qdd = inv(Mq)*(-Cq*qd-Gq+tau);
         
%     % Disturbance turque tau_d
%         taud = 1;
%         tau1 = tau1 + taud;
%         tau1 = tau1 + taud;
       
	% qdd joint acceleration
%         tau_hum = Jtrans*f_hum;
%         qdd(1) = MI11*(-N1 + tau(1)) + MI12*(-N2 + tau(2) + tau_hum(1));
%         qdd(2) = MI12*(-N1 + tau(1)) + MI22*(-N2 + tau(2) + tau_hum(2));
        
% -----------------------------------------------------

    % state equations
    xdot= [ x(3)                                  ; %  1 q1
            x(4)                                  ; %  2 q2
            qdd(1)                                ; %  3 qd1
            qdd(2)                                ; %  4 qd2
            reshape( W_dot', 1, numel(W_dot))'    ; % W = Wdot*dt + W?
            reshape( V_dot , 1, numel(V_dot))'   ]; % V = Vdot*dt + V?

end     

%-------------- END CODE ---------------
