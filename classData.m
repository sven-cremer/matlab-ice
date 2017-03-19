classdef classData
    % For storing controller and simulation data
    
    properties
        % Public properties; underscore, i.e. x_, indicates a function of time
        
        t;      % Time vector
        idx;    % Time index
        
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
        f_hat_;  % NN output
        f_act_;  % Actual value for f_hat_
        fc_;     % Control force
        tau_;    % Control torque
        fc_exp;  % Expected control force
        tau_exp; % Expected control torque
        
        % Robot state
        nJoints;
        x_;    % Cartesian pose
        xd_;   % Cartesian velocity
        q_;    % Joint position
        qd_;   % Joint velocity
        
        ode_t;     % ODE simulation time vector
        ode_x_;    % ODE simulation states
        
        % Human state
        fh_;     % Human force
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
            
            data.idx = 1;
            
            % Preallocate memory
            data.t       = zeros(N,1);
            
            data.normW_  = zeros(N,1);
            data.normV_  = zeros(N,1);
            
            data.lambda_ = zeros(N,nOut);   % Assume diagonal
            data.gamma_  = zeros(N,nOut);
            data.fl_     = zeros(N,nOut);
            
            data.f_hat_  = zeros(N,nOut);
            data.fc_     = zeros(N,nOut);
            data.tau_    = zeros(N,nOut);
            data.fc_exp  = zeros(N,nOut);
            data.tau_exp = zeros(N,nOut);
            
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
        
        function plotVariable(data, str, new_fig, varargin)
            % Plots class variable defined in string str
            
            if( sum( strcmp(str,fieldnames(data)) ) == 0 )
                fprintf('Error: Variable ''%s'' not found!\n',str)
                return
            end
            
            % Check if a new figure should be drawn
            if ~exist('new_fig','var') || isempty(new_fig) || new_fig==1
                figure;
                new_fig = 1;
            else
                hold on;
                % Extract numbers in ylabels for selecting correct subplot
                h_ax = get(gcf,'children');
                J = length(h_ax);
                lab = zeros(J,1); % Mapping from subplot index to label number
                for j=1:J
                    s = get(h_ax(j),'Tag');
                    if( ~isempty(s) )
                        lab(j) = str2num(s);
                    end
                end
            end
            
            if ~exist('varargin','var') || isempty(varargin)
                varargin = {'-b'};
            end
            
            k = data.idx-1;
            
            y = eval(['data.',str,'(1:k,:)']);
            t = data.t(1:k);
            
            N = size(y,2);
            
            if(N == 1)
                
                plot(t, y, varargin{:})
                grid on;
                title(str,'interpreter','none')
                
            else
                
                set(gcf,'position',[75   675   560   840]);
                
                for i=1:N
                    % Select subplot
                    if(new_fig)
                        subplot(N,1,i,'Tag',num2str(i));
                    else
                        idx = 1;
                        for j=1:length(lab) 
                            if lab(j) == i
                                idx = j;
                            end
                        end
                        subplot( h_ax(idx) )
                    end
                    hold on; grid on; 
                    
                    plot(t, y(:,i),varargin{:})
                    %plot(data.t, data.tau_exp_(:,i),':r')
                    
                    xlabel('Time [s]');
                    ylabel(data.getLabel(str,i))
                    if( i == 1 )
                        title(data.getTitle(str))
                    end

                end
            end
            
        end
        
%         function plot(data,varargin)
%             plot(data.t,varargin{:})
%         end
        
%         function disp(data)
%             fprintf(data. ...);
%         end
    end
        
    methods  (Access=private)
        
        function s = getLabel(data, var, idx)
            
            s = [];
            
            if( var(1) == 'x'  || var(1) == 'f' )
                l = {'x [m]','y [m]','z [m]','roll [rad]','pitch [rad]','yaw [rad]'};
                s = l(idx);
                return;
            end
            
            if( var(1) == 'q' )
                s = sprintf('%s%d [rad]',var,idx);
                return;
            end
            
            if( strcmp(var,'tau_') )
                s = sprintf('\\tau_{%d} [rad]',idx);
                return;
            end
            
        end
        
        function s = getTitle(data, var)
            
            s = [];
            
            if( var(1) == 'x' )
                if(var(2) == 'd')
                    s = 'Cartesian Velocity';
                else
                    s = 'Cartesian Pose';
                end               
            elseif( var(1) == 'q' )
                if(var(2) == 'd')
                    s = 'Joint Velocity';
                else
                    s = 'Joint Position';
                end            
            else
                s = strrep(var,'_','\_');
            end
            
        end
        
    end
   
end
