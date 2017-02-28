classdef classNeuroAdaptive
    % Neuroadaptive controller class
    
    properties
        % Public properties
        
        % NN weights
        V;	% Inner weights, [nInp x nHid]
        W;  % Outer weights, [nHid x nOut]
        
        % Controller parameters
        Kv;
        lam;
        F;
        G;
        kappa;
        Kz;
        Zb;
        
        % Output
        f_hat;  % NN output
        fc;     % Control force
        fc_exp; % Expected control force
        
        % Prescribed error dynamics
        Kd;
        Dd;
        gamma;
        lambda;
        fl;
        
        % Flags
        NN_on;      % Use NN output in controller
        PED_on;     % Use prescribed error dybnamics
        RB_on;      % Use robustifying term
        
    end
    
    properties (SetAccess = private)
        % Allow read-only. To change use set methods.
        
        nInp;     % Number of inputs
        nHid;     % Size of hidden layer
        nOut;     % Number of outputs
        
    end
    
    methods     % constructor method
        
        function o = classNeuroAdaptive(nInput, nHidden, nOutput)
            
            o.nInp = nInput;
            o.nHid = nHidden;
            o.nOut = nOutput;
            
            % Initialize random NN weights on the interval [a, b]
            a = -0.1;
            b =  0.1;
            o.W = a + (b-a).*rand( o.nHid, o.nOut );
            o.V = a + (b-a).*rand( o.nInp, o.nHid );
            %o.W = zeros( o.nHid, o.nOut );     <- Could be unstable
            %o.V = zeros( o.nInp, o.nHid );
            
            
            % Controller parameters
            o.Kv     = 10  *eye(o.nOut) ;
            o.lam    = 20  *eye(o.nOut) ;
            o.F      = 100 *eye(o.nHid) ;
            o.G      = 100 *eye(o.nInp) ;
            
            o.kappa  = 0.001;
            o.Kz     = 0.05;
            o.Zb     = 100;
            
            % Output
            o.f_hat  = 0;
            o.fc     = 0;
            o.fc_exp = 0;
                       
            % Prescribed error dynamics
            o.Kd     = 20.0 *eye(o.nOut);
            o.Dd     = 10.0 *eye(o.nOut);
            o.fl     = zeros(o.nOut,1);
            o.gamma  = 9.5 *eye(o.nOut);
            o.lambda = 0.5 *eye(o.nOut);
            
            % Flags          
            o.NN_on  = 1;
            o.PED_on = 1;
            o.RB_on  = 1;
            
        end
        
    end
%{    
    methods    % set/get methods
        
        function o = set.nHid(o, i)
            validateattributes(i, {'integer'}, ...
                {'positive'});
            o.nHid = i;
            %TODO: resize W, V, etc.
        end
        
    end
%}    
    methods     % computational methods
        
        function o = update(o, q, qd, x, xd, x_m, xd_m, xdd_m, fh, dt, robot)
            % Neuroadaptive controller update step (robot is optional)
            
            % Tracking errors
            e  = x_m  - x   ;
            ed = xd_m - xd  ;
            
            % Prescribed Error dynamics
            if(o.PED_on)
                o.gamma = o.Dd - o.lambda;

                lambda_dot = o.Kd - o.gamma * o.lambda;
                o.lambda = o.lambda + lambda_dot * dt;      % TODO: check for convergence?

                fl_dot = fh - o.gamma * o.fl;
                o.fl = o.fl + fl_dot * dt;
                
                r  = ed  + o.lambda*e - o.fl;   % Sliding mode error
            else 
                r  = ed  + o.lam*e ;            % Sliding mode error            
            end
            
            % Robustifying term
            if(o.RB_on)
                Z = [ o.W, zeros(o.nHid, o.nHid)
                    zeros(o.nInp, o.nOut), o.V ];
                v = - o.Kz*(norm(Z) + o.Zb)*r;
            else
                v = zeros(o.nOut,1);
            end
            
            % Input to NN
            y = [ e; ed; x; xd; x_m; xd_m; xdd_m; q; qd ];
            
            % Nonlinear terms
            S = sigmoid(o.V'*y);            % Hidden layer output
            o.f_hat = o.W'*S;
            
            % Control force
            o.fc = o.Kv*r + (o.f_hat - v).*(o.NN_on);
            
            % Expected Control force
            if exist('robot','var') && ~isempty(robot)
                Mx = robot.Mx;
                Cx = robot.Cx;
                Gx = robot.Gx;
                f_act = Mx*( xdd_m + o.lam*ed ) + Cx*( xd_m + o.lam*e ) + Gx;
                o.fc_exp = o.Kv*r + f_act - v;
            else
                o.fc_exp = zeros(o.nOut,1);
            end

            % Parameter update
            if(dt<=0)
                return
            end
            
            % W update
            W_dot = o.F*S*r' - o.kappa*o.F*norm(r)*o.W;
            
            % V update
            sigmoidPrime = diag(S)+diag(S)*diag(S);                 % diag(S)*(I - diag(S)
            V_dot = o.G*y*(sigmoidPrime'*o.W*r)' - o.kappa*o.G*norm(r)*o.V;
            
            % V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) - diag(sigmoid(V'*y))))'*W*r)' ...
            %         - kappa*G*norm(r)*V;
            
            % Update every timestep
            o.W = o.W + W_dot*dt;
            o.V = o.V + V_dot*dt;
                       
        end
        
        function plotWeights(o)
            % Visualizes NN weights
            
            xticks = o.nOut + o.nInp;
            %p = get(gcf,'position');
            %set(gcf,'position',p)

            clim = [-1 1];
            
            h1 = subplot(1,2,1);
            p = get(h1,'position');
            xlength = p(3) * 2 * o.nOut/xticks; % Compute new scaled length
            x_diff = p(3) - xlength;            % Compute difference
            p(3) = xlength;                     % Set new length
            set(h1,'position',p)
            
            imagesc(o.W,clim);
            colormap(cool); %colorbar;
            set(gca,'XTick',1:o.nOut,...
                    'YTick',1:o.nHid,...
                    'TickLength',[0 0]);
            xlabel('Output'); ylabel('Hidden');
            title('W - Outer weights');
            
            h2 = subplot(1,2,2); 
            p = get(h2,'position');      
            p(3) = p(3) * 2 * o.nInp/xticks;    % Compute new scaled length   
            p(1) = p(1) - x_diff;               % Shift subfigure
            set(h2,'position',p)
            
            imagesc(o.V',clim);
            colormap(cool); %colorbar;
            set(gca,'XTick',1:o.nInp,...
                    'YTick',1:o.nHid,...
                    'TickLength',[0 0]);
            ylabel('Hidden');xlabel('Input');
            title('V'' - Inner weights');
            
            %imagesc([na.W, ones(na.nHid,1),na.V'],[-1 1])
            
        end
             
    end
    
    
end