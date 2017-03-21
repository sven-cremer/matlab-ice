classdef classRefTraj
    % Reference trajectory class using Peter Corke's Robotics Toolbox
    % Sven Cremer, 2017
    
    properties
        % Public properties
        
        nJ;     % Number of robot joints
        N;      % Number of trajectory points
        dt;     % Time step
        t;      % Time vector
        tf;     % End time

        x;      % Cartesian reference trajectory
        xd;
        xdd;
        q;      % Joint reference trajectory
        qd;
        
        x0;     % Start pose
        xf;     % Final pose
        q0;     % Start joint position
        qf;     % Final joint position
        
        IKopt;  % Inverse kinematics options
            
    end
    
    methods     % constructor method
        
        function o = classRefTraj(numJoints)
            
            o = reset(o);
            o.nJ = numJoints;
            o.IKopt = 'run'; % Arm right, elbow up, wrist not flipped
            
        end
        
        function o = reset(o)
            % Resets all class variables
            o.N  = 0;
            o.dt = 0.001;
            o.t   = [];
            o.x   = [];
            o.xd  = [];
            o.xdd = [];
            o.q   = [];
            o.qd  = [];
            o.x0 = 0; o.xf = 0;
            o.q0 = 0; o.qf = 0;
            o.tf = 0;
        end
        
    end
    
    methods     % computational methods
        
        function o = updateVariables(o)
            % Update class variables assuming a new x(t) and q(t) has been
            % generated
            if(o.N < 1)
                disp('Initialize a trajectory first!');
                return
            end
            
            % Start and final points
            o.x0 = o.x(1,:);         
            o.q0 = o.q(1,:);
            o.xf = o.x(end,:);
            o.qf = o.q(end,:);
            o.tf = o.t(end);
            
            % Derivatives
            o.xd  = [zeros(1,6); diff(o.x) ]./o.dt;
            o.xdd = [zeros(1,6); diff(o.xd)]./o.dt;
            
            if(isempty(o.qd))
                o.qd  = [zeros(1,o.nJ); diff(o.q)]./o.dt;
            end
            
            m = max(o.xd);
            fprintf('Max velocities: %.3f[m/s], %.3f[rad/s]\n',max(m(1:3)), max(m(4:6)));
        end
            
        function o = circular(o, robot, x0, r0, radius, dt, tf)
            % Circular reference trajectory in yz plane
            % x0 [3x1] starting position
            
            o.nJ = robot.n;
            
            % Time vector
            o.dt = dt;
            o.t  = 0:o.dt:tf;
            o.N  = length(o.t);
            
            % Circle
            c = x0(2:3); % Center (in yz plane)
            p = circle(c, radius,'n',o.N);
            
            % Pose vector
            o.x0 = transl(x0);          
            TC = zeros(4,4,o.N);
            for i=1:o.N
                %TC(:,:,i)= o.x0*trotz(-pi/2)*troty(2*pi*i/o.N)*transl(0, 0, radius);
                TC(:,:,i) = transl(x0(1), p(1,i), p(2,i))*rpy2tr(r0);
            end
            
            xC = transl(TC);
            rC = tr2rpy(TC);
            %rC = unwrap(tr2rpy(TC));
            %rC = repmat(rC(1,:),[o.N,1]);
            o.x   = [xC rC];
            
            % Joint Vector
            o.q  = robot.ikine6s(TC, o.IKopt);  % TODO move? takes a lot of time ...
            
            o = updateVariables(o);
            
        end
        
        function o = straightJtraj(o, robot, x0, r0, x1, r1, dt, tf)
            % Trajectory from x0 to x1 using jtraj function
            o.nJ = robot.n;
            
             % Time vector
            o.dt = dt;
            o.t  = 0:o.dt:tf;
            o.N  = length(o.t);
 
            % Joint vector
            T0 = transl(x0)*rpy2tr(r0);
            T1 = transl(x1)*rpy2tr(r1);
            
            %q0 = robot.ikine(T0);
            %q1 = robot.ikine(T1);         
            q0 = robot.ikine6s(T0, o.IKopt);
            q1 = robot.ikine6s(T1, o.IKopt);
            
            [o.q, o.qd] = jtraj(q0, q1, o.N ); 
            
            % Pose vector
            TC = robot.fkine(o.q);
            xC = transl(TC);
            rC = unwrap( tr2rpy(TC) );
            o.x = [xC rC];     % TODO check for discontinuity in rC
            
            o = updateVariables(o);
        end
        
        function o = straightCtraj(o, robot, x0, r0, x1, r1, dt, tf)
            % Trajectory from x0 to x1 using ctraj function
            o.nJ = robot.n;
            
            % Time vector
            o.dt = dt;
            o.t  = 0:o.dt:tf;
            o.N  = length(o.t);
            
            % Pose vector
            T0 = transl(x0)*rpy2tr(r0);
            T1 = transl(x1)*rpy2tr(r1);
            
            TC = ctraj(T0, T1, o.N );         
            xC = transl(TC);
            rC = tr2rpy(TC);
            o.x = [xC rC];

            % Joint vector
            o.q = robot.ikine6s(TC,o.IKopt);

            o = updateVariables(o);
        end
        
        function o = holdEndpoints(o,dtStart,dtStop)
            % Hold endpoints
            if(o.N < 1)
                disp('Initialize a trajectory first!');
                return
            end
            
            qz = zeros(1,o.nJ);
            xz = zeros(1,6);
            
            % Data points
            n0 = round(dtStart/o.dt);
            nf = round(dtStop/o.dt);
            
            % Start          
            x_0  = repmat(o.x0, [n0 1]);
            xd_0 = repmat(xz,   [n0 1]);          
            q_0  = repmat(o.q0, [n0 1]);
            qd_0 = repmat(qz,   [n0 1]);
            
            % Stop
            x_f  = repmat(o.xf, [nf 1]);
            xd_f = repmat(xz,   [nf 1]);         
            q_f  = repmat(o.qf, [nf 1]);
            qd_f = repmat(qz,   [nf 1]);
            
            % Extend trajectories
            o.x   = [x_0;  o.x;   x_f];
            o.xd  = [xd_0; o.xd;  xd_f];
            o.xdd = [xd_0; o.xdd; xd_f];
            
            o.q   = [q_0;  o.q;   q_f];
            o.qd  = [qd_0; o.qd;  qd_f];
            
            o.t = linspace(0, o.t(end)-o.t(1)+dtStart+dtStop, o.N+n0+nf)';
            o.N = length(o.t);
            o.tf = o.t(end);
            
        end
        
    end
    
    methods     % for visualization
        
        function plotTraj(o)
            % Visualize robot trajectory
            
            p = [75   675   560   840];
            
            h = figure;
            set(h, 'position', p);
            
            subplot(2,1,1)
            plot(o.t,o.x(:,1:3))
            xlabel('Time [s]'); ylabel('Position [m]');
            legend('x','y','z');
            grid on;
            
            subplot(2,1,2)
            plot(o.t,o.x(:,4:6))
            xlabel('Time [s]'); ylabel('Rotation [rad]');
            legend('roll','pitch','yaw');
            grid on;
            
        end
        
        function animateTraj(o,robot)
            
            % Compute q(t) with a fixed time step
            simStep = 0.01;
            t_int   = (o.t(1):simStep:o.t(end))';
            q_int   = interp1(o.t, o.q, t_int);
            
            figure
            w = 0.75;
            W = [-w, w -w w -0.2 1.0];
            robot.plot(q_int,'trail','-m','delay',0.1*simStep, ...
                       'workspace',W,'scale',0.7);
            fprintf('Done!\n');
            
        end
             
    end
end