classdef classData
    % For storing controller and simulation data
    
    properties
        % Public properties; underscore, i.e. x_, indicates a function of time
        
        t;  % Time vector
        
        % NA weights and controller parameters
        normW_;
        normV_;
        %V_;	% Inner weights, [nInp x nHid]
        %W_;    % Outer weights, [nHid x nOut]
             
        % Prescribed error dynamics
        Kd;
        Dd;
        gamma_;
        lambda_;
        fl_;
        
        % Output
        %f_hat_;  % NN output
        fc_;     % Control force
        tau_;    % Control torque
        %fc_exp;  % Expected control force
        
        % Robot state
        nJoints;
        x_;    % Cartesian pose
        xd_;   % Cartesian velocity
        q_;    % Joint position
        qd_;   % Joint velocity
        
        ode_t;      % ODE simulation time vector
        ode_x_;    % ODE simulation states
        
        % Human state
        f_h_;    % Human force
        x_h_;    % Cartesian pose
        xd_h_;   % Cartesian velocity
        
        % Reference trajectory
        x_m_;
        xd_m_;
        %xdd_m_t;
        
    end
        
    methods     % constructor method
        
        function data = classData(N, nOut, nJoints)
            % N is the number of expected data samples
            
            if ~exist('N','var') || isempty(N)
                N = 1;
            end
            if ~exist('nOut','var') || isempty(nOut)
                nOut = 6;
            end
            if ~exist('nJoints','var') || isempty(nJoints)
                data.nJoints = nOut;
            else
                data.nJoints = nJoints;
            end
            
            % Preallocate memory
            data.t       = zeros(N,1);
            
            data.normW_  = zeros(N,1);
            data.normV_  = zeros(N,1);
            
            data.lambda_ = zeros(N,nOut*2);
            data.gamma_  = zeros(N,nOut*2);
            data.fl_     = zeros(N,nOut);
            
            %data.f_hat_  = zeros(N,numOut);
            data.fc_     = zeros(N,nOut);
            data.tau_    = zeros(N,nOut);
            %data.tau_exp = zeros(N,numOut);
            
            data.x_      = zeros(N,nOut);
            data.xd_     = zeros(N,nOut);
            data.q_      = zeros(N,nOut);
            data.qd_     = zeros(N,nOut);
            
            data.fh_     = zeros(N,1);           
            data.x_h_    = zeros(N,1);
            data.xd_h_   = zeros(N,1);
            
            data.x_m_    = zeros(N,nOut);
            data.xd_m_   = zeros(N,nOut);
            %data.xdd_m_  = zeros(N,numOut);
            
            data.ode_t   = []; % Unknown size
            data.ode_x_  = [];
             
        end
        
    end
  
    methods     % data analysis
               
        function plotJoints(data, t_sim, q_sim,  qd_sim)
            
            figure;
            set(gcf,'position',[75   675   560   840]);
            
            for i=1:data.nJoints
                subplot(data.nJoints,1,i)
                hold on; grid on;
                
                plot(t_sim, q_sim(:,i),'-b')
                plot(data.t, data.q_(:,i),':r')
                
                xlabel('Time [s]');
                ylabel(sprintf('Joint %d [rad]',i))
                suptitle('JOINT POSITION')
            end
            
            figure;
            set(gcf,'position',[675   675   560   840]);
            
            for i=1:data.nJoints
                subplot(data.nJoints,1,i)
                hold on; grid on;
                
                plot(t_sim, qd_sim(:,i),'-b')
                plot(data.t, data.qd_(:,i),':r')
                
                xlabel('Time [s]');
                ylabel(sprintf('Joint %d [rad]',i))
                suptitle('JOINT VELOCITY')
            end
            
        end
             
    end
    
end
