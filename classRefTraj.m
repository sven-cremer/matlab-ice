classdef classRefTraj
    % Reference trajectory class using Peter Corke's Robotics Toolbox
    % Sven Cremer, 2017
    
    properties
        % Public properties
        
        N;      % Number of trajectory points
        %x0;
        %q0;
        
        %t0;
        %tf;
        
    end
    
    methods     % constructor method
        
        function o = classRefTraj()
            
            o.N = 100;
            
        end
        
    end
    
    methods     % computational methods
           
        
        function xref = circular(o, x0, r0, radius, N)
            % Circular reference trajectory
            % x0 [3x1]
            
            x0 = transl(x0);
            
            TC = [];
            for i=1:N
                TC(:,:,i)= x0*trotz(-pi/2)*troty(2*pi*i/N)*transl(0, 0, radius);
            end
            
            xC = transl(TC);
            rC = tr2rpy(TC);
            xref = [xC rC];
        end
        
        function [xref, q, qd] = straightJtraj(o, robot, x0, r0, x1, r1, N)
            
            T0 = transl(x0)*rpy2tr(r0);
            T1 = transl(x1)*rpy2tr(r1);
            
            q0 = robot.ikine(T0);
            q1 = robot.ikine(T1);
            
            [q, qd] = jtraj(q0, q1, N ); 
            
            TC = robot.fkine(q);
          
            xC = transl(TC);
            rC = tr2rpy(TC);
            xref = [xC rC];     % TODO check for discontinuity in rC
        end
        
        function xref = straightCtraj(o, robot, x0, r0, x1, r1, N)
            
            T0 = transl(x0)*rpy2tr(r0);
            T1 = transl(x1)*rpy2tr(r1);
            
            TC = ctraj(T0, T1, N );
          
            xC = transl(TC);
            rC = tr2rpy(TC);
            xref = [xC rC];
        end
        
        function xt_new = holdEndpoints(o,xt,dtStart,dtStop)
            
            t    = xt(:,1);
            xref = xt(:,2:7);
            
            N = length(t);
            dt = median(diff(t));
            
            % Start
            x0 = xref(1,1:3);
            r0 = xref(1,4:6);
            T0 = transl(x0)*rpy2tr(r0);
            n0 = round(dtStart/dt);
            
            % Stop
            xf = xref(end,1:3);
            rf = xref(end,4:6);
            Tf = transl(xf)*rpy2tr(rf);
            nf = round(dtStop/dt);
            
            % Create trajectories
            T_0 = ctraj(T0, T0, n0  ); 
            T_f = ctraj(Tf, Tf, nf  ); 
            
            x_0 = [transl(T_0) tr2rpy(T_0)];
            x_f = [transl(T_f) tr2rpy(T_f)];
            
            xref_new = [x_0; xref; x_f];
            t_new = linspace(0, t(end)-t(1)+dtStart+dtStop, N+n0+nf)';
            
            xt_new = [t_new xref_new];
            
        end
    end
    
    methods     % for visualization
        
        function plotTraj(o, robot)
            % Visualize robot trajectory
            
            
        end
             
    end
end