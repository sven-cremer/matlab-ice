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
% COMPUTE CONTROL INPUT FOR ROBOT ARM
% -----------------------------------------------------

% -----------------------------------------------------
% STATES

    % Robot states        
        q  = [x(1) x(2)]';
        qd = [x(3) x(4)]';
        
    % arm parameters 
    m1= 0.8 ; m2= 2.3 ; a1= 1 ; a2= 1 ; g= 9.8 ; % arm parameters
        
    % Robot Jacobian
         J = [ - sin(q(1))*a1 - a2*sin(q(1) + q(2)), - a2*sin(q(1) + q(2))  ;
                 cos(q(1))*a1 + a2*cos(q(1) + q(2)),   a2*cos(q(1) + q(2)) ];
            
        xC = [ a1*cos(q(1)) + a2*cos(q(1) + q(2))  ;
               a1*sin(q(1)) + a2*sin(q(1) + q(2)) ];
        xCd= J*qd;
           
    % Jacobian inverse
        Jinv = pinv(J);

%     qd  = Jinv*yd;    
            
    % Jacobian transpose
        Jtrans = J';
        
    % Jacobian transpose inverse
        JtransInv = pinv(Jtrans);
        
    % Jacobian dot    
        Jdot = [ - cos(q(1))*a1*qd(1) - a2*cos(q(1) + q(2))*(qd(1) + qd(2)), ...
                 - a2*cos(q(1) + q(2))*(qd(1) + qd(2))  ;
                 
                 -sin(q(1))*a1*qd(1) - a2*sin(q(1) + q(2))*(qd(1) + qd(2)) , ...
                 -a2*sin(q(1) + q(2))*(qd(1) + qd(2)) ];

% -----------------------------------------------------
% MODEL

    % Model states

        % Discrete model states
        global q_m_z   ;
        global qd_m_z  ;
        global qdd_m_z ;
                
        q_m   = q_m_z   ;
        qd_m  = qd_m_z  ;
        qdd_m = qdd_m_z ;

% -----------------------------------------------------
% CONTROLLER

    % NN control input
    
    % controller parameters
        input  = x(5)            ; % 5 input  18 
        output = x(6)            ; % 6 output 2
        hidden = x(7)            ; % 7 hidden 20 
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
        nnStart = 8;
        
        endW   = (nnStart + hidden*output-1);
        startV = endW + 1;
        endV   = startV + input*hidden - 1;
        W      = reshape(x(nnStart:endW    ), output, hidden)';
        V      = reshape(x(startV:endV), input , hidden) ;
        
    % robustifying term
        Z    = [ W zeros(size(W, 1), size(V, 2)); zeros(size(V, 1), size(W, 2)) V ];
        v    = - Kz*(norm(Z) + Zb)*r;
        
    % Computed torques
    
        % inertia M(q) and nonlinear terms N(q, qdot)
            M11 =  (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(2)) ;
            M12 =  m2*a2^2 + m2*a1*a2*cos(x(2)) ;
            M22 =  m2*a2^2 ;
            N1  = -m2*a1*a2*(2*x(3)*x(4) + x(4)^2)*sin(x(2)) ;
            N1  =  N1 + (m1 + m2)*g*a1*cos(x(1)) + m2*g*a2*cos(x(1) + x(2)) ;
            N2  =  m2*a1*a2*x(3)^2*sin(x(2)) + m2*g*a2*cos(x(1) + x(2)) ;

%        % PD CT control torques
%             f_true = M11*s1 + M12*s2 + N1 ;
%             f_true = M12*s1 + M22*s2 + N2 ;

    % control torques
    
        % Input to NN
        y = [ e; ep; xC; xCd; q_m; qd_m; qdd_m; q; qd ];
        
        % Control output
        f_hat = W'*sigmoid(V'*y);

        % Perfect f_hat
        jM = [ M11 M12 ;
               M12 M22 ];
        jC = [ -m2*a1*a2*(2*x(3)*x(4) + x(4)^2)*sin(x(2))  ;
               m2*a1*a2*x(3)^2*sin(x(2))                  ];
           
        jCm= [ 0, -m2*a1*a2*(2*x(3) + x(4))*sin(x(2)) ;
               m2*a1*a2*x(3)*sin(x(2)), 0            ];

        jG = [ (m1 + m2)*g*a1*cos(x(1)) + m2*g*a2*cos(x(1) + x(2))  ;
               m2*g*a2*cos(x(1) + x(2))                            ];
           
        Mx = JtransInv*jM*Jinv;
        Cx = JtransInv*( jCm - jM*Jinv*Jdot )*Jinv;
        Gx = JtransInv*jG;
        
        % This is the real function to be approximated
        f_rel = Mx*( qdd_m + lam*ep ) + Cx*( qd_m + lam*e ) + Gx;
%         f_hat = f_rel;
 
         %tau  =  Jtrans*( Kv*r + f_hat - v );
         tau  =  Jtrans*f_rel;
%         tau  =  Jtrans*( Kv*r - f_hum + f_hat - v );

    % parameter updates
        % V update
        W_dot = F*sigmoid(V'*y)*r' - kappa*F*norm(r)*W;
        
        % W update
        V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) -...
                diag(sigmoid(V'*y))))'*W*r)'  - kappa*G*norm(r)*V;
           
% -----------------------------------------------------
% ROBOT ARM DYNAMICS SIMULATION
% -----------------------------------------------------

%     m1= 1 ; m2= 1 ; a1= 1 ; a2= 1 ; g= 9.8 ;    % arm parameters
    
    % inertia M(q) and nonlinear terms N(q, qdot)
        M11 =  (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(x(2)) ;
        M12 =  m2*a2^2 + m2*a1*a2*cos(x(2)) ;
        M22 =  m2*a2^2 ;
        N1  = -m2*a1*a2*(2*x(3)*x(4) + x(4)^2)*sin(x(2)) ;
        N1  =  N1 + (m1 + m2)*g*a1*cos(x(1)) + m2*g*a2*cos(x(1) + x(2)) ;
        N2  =  m2*a1*a2*x(3)^2*sin(x(2)) + m2*g*a2*cos(x(1) + x(2)) ;
        
    % Inversion of M(q) (for large values of n, use least-squares)
        det  =  M11*M22 - M12*M12 ;
        MI11 =  M22 / det ;
        MI12 = -M12 / det ;
        MI22 =  M11 / det ;
        
%     % Disturbance turque tau_d
%         taud = 1;
%         tau1 = tau1 + taud;
%         tau1 = tau1 + taud;
       
	% qdd joint acceleration
%         tau_hum = Jtrans*f_hum;
%         qdd(1) = MI11*(-N1 + tau(1)) + MI12*(-N2 + tau(2) + tau_hum(1));
%         qdd(2) = MI12*(-N1 + tau(1)) + MI22*(-N2 + tau(2) + tau_hum(2));
        
        % No human force
        qdd(1) = MI11*(-N1 + tau(1)) + MI12*(-N2 + tau(2) ); 
        qdd(2) = MI12*(-N1 + tau(1)) + MI22*(-N2 + tau(2) );

% -----------------------------------------------------

    % state equations
    xdot= [ x(3)                                  ; %  1 q1
            x(4)                                  ; %  2 q2
            qdd(1)                                ; %  3 qd1
            qdd(2)                                ; %  4 qd2
            0                                     ; %  5 input  
            0                                     ; %  6 output 
            0                                     ; %  7 hidden
            reshape( W_dot', 1, numel(W_dot))'    ; % 
            reshape( V_dot , 1, numel(V_dot))'   ];

end     

%-------------- END CODE ---------------
