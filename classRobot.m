classdef classRobot
    % Robot class
    
    properties
        % Public properties
        
        x;          % Cartesian position
        xd;         % Cartesian velocity
        q;          % Joint positino
        qd;         % Joint velocity
        
        fc;         % Cartesian force
        tau;        % Applied torque
        tau_max;    % Maximum torque
        tau_min;    % Minimum torque
        
        J;          % Jacobian
        Jdot;       % Jacobian derivative
        Jt;         % Jacobian transpose
        Jinv;       % Jacobian pseudo-inverse
        
        gravity = 9.81;
        
        % Geometry
        length;
        mass;
        
        % Dynamics
        Mq; Cq; Gq;
        Mx; Cx; Gx;   
        
    end
    
    properties (SetAccess = private)
        % Allow read-only. To change use set methods.
        
        nJoints;    % Number of joints
        
    end
    
    methods     % constructor method
        
        function o = classRobot(nJoints)
            
            o.nJoints = nJoints;
            
            o.length = ones(nJoints,1);
            o.mass   = ones(nJoints,1);
            
            o.tau_max = 100;
            o.tau_min = -100;
                       
           if(o.nJoints ~= 2)
               disp('Warn: Most functions assume 2 DOF!');
           end
            
        end
        
    end
  
    methods     % computational methods
        
        function q = IK(o, x)
            % Inverse Kinematics (assuming 2 DOF)
            
            q = zeros(o.nJoints,1);
            
            % Arm parameters
            a1 = o.length(1);
            a2 = o.length(2);
            
            c2 = (x(1)^2 + x(2)^2 - a1^2 - a2^2)/(2*a1*a2);
            s2 = sqrt(1 - c2.^2);   % plus/minus
            
            q(2) = atan2(s2, c2);
            q(1) = atan2(x(2), x(1)) - atan2(a2*s2, a1 + a2*c2);                      
        end
        
        function x = FK(o, q)
            % Forward Kinematics (assuming 2 DOF)
           
            % Arm parameters
            a1 = o.length(1);
            a2 = o.length(2);
            
            % Analytical Jacobian
            x = [ a1*cos(q(1)) + a2*cos(q(1) + q(2))  ;
                a1*sin(q(1)) + a2*sin(q(1) + q(2)) ];        
        end
        
        function J = Jacobian(o, q)
            % Analytical Jacobian (assuming 2 DOF)
            
            % Arm parameters
            a1 = o.length(1);
            a2 = o.length(2);
            
            J = [ - sin(q(1))*a1 - a2*sin(q(1) + q(2)), - a2*sin(q(1) + q(2))  ;
                cos(q(1))*a1 + a2*cos(q(1) + q(2)),   a2*cos(q(1) + q(2)) ];
            
            % Check J
            if( sum(isnan(J(:))) > 0 )
                disp('Error: Jacobian contains NaN values');
            end
            
        end
        
        function Jdot = JacobianDot(o, q, qd)
            
            % Arm parameters
            a1 = o.length(1);
            a2 = o.length(2);
            
            % Jacobian dot
            Jdot = [ - cos(q(1))*a1*qd(1) - a2*cos(q(1) + q(2))*(qd(1) + qd(2)), ...
                     - a2*cos(q(1) + q(2))*(qd(1) + qd(2))  ;
                
                     -sin(q(1))*a1*qd(1) - a2*sin(q(1) + q(2))*(qd(1) + qd(2)) , ...
                     -a2*sin(q(1) + q(2))*(qd(1) + qd(2)) ];
            
        end
        
        function robot = updateState(robot, q, qd)
            
            % Jacobians
            robot.J     = Jacobian(robot, q);
            robot.Jt    = robot.J';
            robot.Jinv  = pinv(robot.J);
            robot.Jdot  = JacobianDot(robot, q, qd);
            JtransInv   = pinv(robot.Jt);   % Transpose inverse
            
            % Kinematics
            robot.q  = q;
            robot.qd = qd;
            
            robot.x  = FK(robot, q);
            robot.xd = robot.J * qd;
            
            % Dynamics - Joint space
            [Mq,Cq,Gq] = dynamicsJoint(robot, q, qd);
            robot.Mq = Mq;
            robot.Cq = Cq;
            robot.Gq = Gq;
            
            % Dynamics - Cartesian space
            robot.Mx = JtransInv*Mq*robot.Jinv;
            robot.Cx = JtransInv*( Cq - Mq*robot.Jinv*robot.Jdot )*robot.Jinv;
            robot.Gx = JtransInv*Gq;     
            
        end
        
        function [Mq,Cq,Gq] = dynamicsJoint(o, q, qd)
            % Robot dynamics (assuming 2 DOF)
            
            % Arm parameters
            m1 = o.mass(1);       % Center of mass is located at a1, i.e. ac1=a1
            m2 = o.mass(2);
            a1 = o.length(1);
            a2 = o.length(2);
            g  = o.gravity;
            
            % Input variables
            q1 = q(1);
            q2 = q(2);
            dq1 = qd(1);
            dq2 = qd(2);
            
            % Inertia term M(q)
            M11 = (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(q2) ;
            M12 = m2*a2^2 + m2*a1*a2*cos(q2) ;
            M22 = m2*a2^2 ;
            Mq = [M11, M12; M12, M22];
            
            % Coriolis term C(q,dq)
            h = -m2*a1*a2*sin(q2);
            C11 = h*dq2 ;
            C12 = h*(dq1+dq2) ;
            C21 = -h*dq1;
            C22 = 0;
            Cq = [C11, C12; C21, C22];
            
            % Gravity term G(q)
            G1 = (m1 + m2)*g*a1*cos(q1) + m2*g*a2*cos(q1+q2);
            G2 = m2*g*a2*cos(q1 + q2);
            Gq = [G1; G2];
            
        end
        
        function [Mx,Cx,Gx] = dynamicsCartesian(o, q, qd)
            
            % Analytical Jacobian
            J = Jacobian(o, q);

            % Jacobians
            Jinv = pinv(J);             % Inverse
            Jtrans = J';                % Transpose
            JtransInv = pinv(Jtrans);   % Transpose inverse
            
            % Jacobian dot
            Jdot = JacobianDot(o, q, qd);
            
            % Joint space dynamics
            [Mq,Cq,Gq] = dynamicsJoint(o, q, qd);
            
            % Cartesian space dynamics
            Mx = JtransInv*Mq*Jinv;
            Cx = JtransInv*( Cq - Mq*Jinv*Jdot )*Jinv;
            Gx = JtransInv*Gq;     
            
        end
        
        function tau_sat = torqueSaturation(o, tau)
            % Applies torque saturation
            
            tau_sat = tau;
            
            % Option 1: cutoff values
            %tau_sat( tau_sat > o.tau_max ) = o.tau_max;
            %tau_sat( tau_sat < o.tau_min ) = o.tau_min;
                     
            % Option 2: Scale all values
            idxMax = tau_sat > o.tau_max;
            idxMin = tau_sat < o.tau_min;
            if( sum(idxMax(:)) + sum(idxMin(:)) > 0)
                r1 = o.tau_max / max( tau(:) );
                r2 = o.tau_min / min( tau(:) );
                if(r1 < r2)
                    tau_sat = tau .* r1;
                else
                    tau_sat = tau .* r2;
                end
            end
  
        end
        
        
        function xdot = odeSim(t, x, robot)
            % Function for ODE solver, assumes robot.tau is up-to-date
            
            % Error checking
            idxNaN = isnan(x);
            if( sum( idxNaN(:) ) > 0 )
                disp('Error: x contains NaN values inside odeSim()');
                x(idxNaN) = 0;
            end
            
            % ROBOT STATE
            tau_c = robot.tau;
            
            q  = [x(1) x(2)]';
            qd = [x(3) x(4)]';
            
            % ROBOT ARM DYNAMICS SIMULATION
            
            [Mq,Cq,Gq] = dynamicsJoint(robot, q, qd);
            
            % Arm dynamics
            qdd = inv(Mq)*(-Cq*qd-Gq+tau_c);
            
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
        
        
             
    end
    
    
end
