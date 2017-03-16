classdef classDigitalPID
    % Digital PID controller class
    % Sven Cremer, 2017
    
    properties
        % Public properties
               
        % Controller parameters
        Kp;     % Proportional gain
        Ki;     % Integral gain
        Kd;     % Derivative gain
        Ts;     % Sampling time PID
        N;      % Filtering coefficient for derivative term

        % Output
        fc;     % Control force
        fd;     % Desired force
        fd_delay;   % Delay desired input force
        
    end
    
    properties (SetAccess = private)
        % Allow read-only. To change use set methods.
        
        lastUpdate;
        ku1; ku2;
        ke0; ke1; ke2;
        
        e0; e1; e2;
        u0; u1; u2;
        
    end
    
    methods     % constructor method
        
        function o = classDigitalPID(Kp, Ki, Kd, Ts, N)
            
            % Controller parameters
            o.Kp = Kp;
            o.Ki = Ki;
            o.Kd = Kd;
            o.Ts = Ts;
            o.N  = N;
            
            % Backwards Euler
            a0 = (1+N*Ts);
            a1 = -(2 + N*Ts);
            a2 = 1;
            b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
            b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
            b2 = Kp + Kd*N;
            
            o.ku1 = a1/a0;
            o.ku2 = a2/a0;
            o.ke0 = b0/a0;
            o.ke1 = b1/a0;
            o.ke2 = b2/a0;
            
            o.e0 = 0; o.e1 = 0; o.e2 = 0;
            o.u0 = 0; o.u1 = 0; o.u2 = 0;
            
            % Output
            o.fc  = 0;
            o.fd  = 0;
            
            o.fd_delay = 0;    
            o.lastUpdate = 0;
            
        end
        
    end
   
    methods     % computational methods
        
        function o = update(o, t, fm)
            % Controller update step
            % fm = fmeasured
            
            if( t - o.lastUpdate >= o.Ts )
                
                % Update variables
                o.e2=o.e1; o.e1=o.e0;
                o.u2=o.u1; o.u1=o.u0;
                
                % Compute new error
                if(t < o.fd_delay)
                    fd = 0;
                else
                    fd = o.fd;
                end
                o.e0 = fd - fm;
                %o.e0 = fm + fd;
                
                % Compute new control
                o.u0 = -o.ku1*o.u1 - o.ku2*o.u2 + o.ke0*o.e0 + o.ke1*o.e1 + o.ke2*o.e2;
                
                % Store results
                o.fc = o.u0;
                
                % Update last time
                o.lastUpdate = t;
            else
                % Use previous value
            end
               
        end
        
        function printCoefficients(o)
            
            fprintf('u0 = - %.2fu1 - %.2fu2 + %.2fe0 + %.2fe1 + %.2fe2\n', o.ku1, o.ku2, o.ke0, o.ke1, o.ke2);
            
        end
        
             
    end
    
    
end
