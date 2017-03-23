classdef classNeuroAdaptive
    % Neuroadaptive controller class   
    % Sven Cremer, 2017
    
    properties
        % Public properties
        
        nJoints;    % Number of joints
        nCart;      % Number of Cartesian coordinates, 7 implies the use of 
                    % quaternions instead of Euler angles
        
        % NN weights
        V;	% Inner weights, [nInp x nHid]
        W;  % Outer weights, [nHid x nOut]
        
        b;  % Bias unit (0 or 1)
        
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
        f_act;  % Actual value for f_hat
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
        
        actFncs;    % Structure with activation funcitons
        actF;       % Selected activation function
        
        rbf_mu;     % RBF center for each node n [nInp x nNodes]
        rbf_beta;   % RBF beta = 1/(2*sigma^2)
        
    end
    
    properties (SetAccess = private)
        % Allow read-only. To change use set methods.
        
        nInp;     % Number of inputs
        nHid;     % Size of hidden layer
        nOut;     % Number of outputs
        
    end
    
    methods     % constructor method
                
        function o = classNeuroAdaptive(nJoints, nCart, nHidden)
            % nJoints : Number of robot joints
            % nCart   : Number of Cartesian coordinates of the input and
            %           error signals, for example
            %            2 -> xy plane without rotation
            %            6 -> x,y,z and roll,pitch,yaw
            %            7 -> x,y,z and quaternion rotation
            %           Also determines the number of NN output nodes.
            % nHidden : Number of nodes in the NN hidden layer.
            
            o.nJoints = nJoints;
            o.nCart   = nCart;
            
            o.nInp = nJoints*2 + nCart*7;   % Depends on NN input vector y
            o.nHid = nHidden;
            o.nOut = nCart;
            
            o.b = 1;    % Bias unit
            
            % Initialize random NN weights on the interval [a, b]
            %rng(0)
            a = -0.5;
            b =  0.5;
            o.W = a + (b-a).*rand( o.nHid+o.b, o.nOut );
            %o.V = a + (b-a).*rand( o.nInp, o.nHid );
            %o.W = zeros( o.nHid, o.nOut );     %<- Could be unstable?
            o.V = zeros( o.nInp+o.b, o.nHid);
            
            
            % Controller parameters
            o.Kv     = 10  *eye(o.nCart) ;
            o.lam    = 20  *eye(o.nCart) ;
            o.F      = 100 *eye(o.nHid+o.b) ;
            o.G      = 100 *eye(o.nInp+o.b) ;
            
            o.kappa  = 0.001;
            o.Kz     = 0.05;
            o.Zb     = 100;
            
            % Output
            o.f_hat  = 0;
            o.fc     = 0;
            o.fc_exp = 0;
                       
            % Prescribed error dynamics
            o.Kd     = 20.0 *eye(o.nCart);
            o.Dd     = 10.0 *eye(o.nCart);
            o.fl     = zeros(o.nCart,1);
            o.gamma  = 9.5 *eye(o.nCart);
            o.lambda = 0.5 *eye(o.nCart);
            
            % Flags          
            o.NN_on  = 1;
            o.PED_on = 1;
            o.RB_on  = 1;
            
            % Activation function
            o.actFncs = struct('Sigmoid',1,'Tanh',2,'RBF',3); 
            o.actF = o.actFncs.RBF;
            
            % For sigma(z)
            %   if z=y   is [nInp x 1] then rbf_mu is [nInp x nHid]
            %   if z=V'y is [nHid x 1] then rbf_mu is [nHid x nHid]
            % If the centers are -1 or 1, then there are 2^(nHid) possible
            % combinations. This would require too many hidden layer nodes.
            %{
            if(o.nHid <= 10)
                Mu = ones(o.nHid,1);  % First possibility
                for i=1:o.nHid
                    v = ones(o.nHid,1);
                    v(1:i) = -1;
                    P = uperm(v)';
                    Mu = [Mu, P];
                end
                N = size(Mu,2);
                idx = randperm(N,o.nHid);   % Select nHid Mu(:,i) randomly
                o.rbf_mu = Mu(:,idx);
            else                
                o.rbf_mu = randi([0 1], o.nHid, o.nHid );   % Uniform discrete distribution (0 or 1)
                o.rbf_mu(o.rbf_mu == 0) = -1;               % Make elements -1 or 1
            end
            %}
            %o.rbf_mu = rand( o.nHid, o.nHid ); % Unstable, needs many nodes ?
            o.rbf_mu = randi([0 1], o.nHid, o.nHid ); % [0,+/-1], [-1,0,1]  perform similarly
            o.rbf_mu(o.rbf_mu == 0) = -1; % Best result

            % Compute average distance between centers mu_j
            D = dist(o.rbf_mu); % The Euclidean distance between two vectors Mu(:,i) and Mu(:,j) is calculated as D(i,j)
            idx = tril(true(size(D)),-1);   % Extract lower triangular part
            d_avg = mean( D(idx) );         % Overall (?) average distance
            o.rbf_beta = 1/(2*(2*d_avg)^2); % sigma = 2*d_avg
            fprintf('beta1 = %f\n', o.rbf_beta)
            o.rbf_beta = max(D(:))/sqrt(2*o.nHid);  % dmax/sqrt(2*M)
            fprintf('beta2 = %f\n', o.rbf_beta)
          
            o.rbf_beta = 1.0;
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
                Z = [ o.W, zeros(o.nHid+o.b, o.nHid)
                    zeros(o.nInp+o.b, o.nOut), o.V ];
                v = - o.Kz*(norm(Z) + o.Zb)*r;
            else
                v = zeros(o.nOut,1);
            end
            
            % Input to NN
            %y = [ e; ed; x; xd; x_m; xd_m; xdd_m; q; qd ];
            %y = [ o.fl; fl_dot; diag(o.lambda); diag(lambda_dot); q; qd; e; ed; x_m; xd_m];
            y = [ diag(o.lambda); diag(lambda_dot); q; qd; e; ed; x_m; xd_m; xdd_m];
            %y= [q; qd; e; ed; x_m; xd_m];
            if(o.b)
                y = [y;1];
            end
            
            % Nonlinear terms
            S = o.activation(o.V'*y);           % Hidden layer output
            o.f_hat = o.W'*S;
            
            % Control force
            o.fc = o.Kv*r + (o.f_hat - v).*(o.NN_on);
            
            % Expected Control force
            if exist('robot','var') && ~isempty(robot)
                Mx = robot.Mx;
                Cx = robot.Cx;
                Gx = robot.Gx;
                o.f_act = Mx*( xdd_m + o.lam*ed ) + Cx*( xd_m + o.lam*e ) + Gx;
                o.fc_exp = o.Kv*r + o.f_act - v;
            else
                o.f_act  = zeros(o.nOut,1);
                o.fc_exp = zeros(o.nOut,1);
            end

            % Parameter update
            if(dt<=0)
                return
            end
            
            % W update
            W_dot = o.F*S*r' - o.kappa*o.F*norm(r)*o.W;
            
            % V update
            dS = o.activationPrime(o.V'*y);
            V_dot = o.G*y*(dS'*o.W*r)' - o.kappa*o.G*norm(r)*o.V;
            
            % V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) - diag(sigmoid(V'*y))))'*W*r)' ...
            %         - kappa*G*norm(r)*V;
            
            % Update every timestep
            o.W = o.W + W_dot*dt;
            o.V = o.V + V_dot*dt;
                       
        end
        
        
        function g = activation(o, z)
            % Activation function           
            switch(o.actF)
                
                case o.actFncs.Sigmoid
                    g = 1./(1+exp(-z));
                    
                case o.actFncs.RBF
                    g = zeros(o.nHid,1);
                    for i=1:o.nHid
                        g(i,1) = exp( - o.rbf_beta * norm(z-o.rbf_mu(:,i))^2 ); % Note: ||x||^2 is the same as (x'*x)
                    end
                    
                otherwise
                    fprintf('Invalid activation function!\n');
                    g = [];
            end
            if(o.b)
                g = [g; 1];
            end
        end
        
        function g = activationPrime(o, z)
            % Derivative of activation function           
            switch(o.actF)
                
                case o.actFncs.Sigmoid
                    S = 1./(1+exp(-z));             %activation(o,z);
                    g = diag(S)+diag(S)*diag(S);    % diag(S)*(I - diag(S))

                case o.actFncs.RBF
                    g = zeros(o.nHid,o.nHid);
                    for i=1:o.nHid
                        x = z-o.rbf_mu(:,i);
                        g(:,i) = -2*o.rbf_beta*abs(x)*exp( - o.rbf_beta * norm(x)^2 ); % TODO row or col?
                    end
                    
                otherwise
                    fprintf('Invalid activation function!\n');
                    g = [];
            end
            if(o.b)
                g = [g; zeros(1,o.nHid)];
            end
        end
        
    end
    
    methods     % for visualization
        
        function plotWeights(o, clim)
            % Visualizes NN weights
            
            xticks = o.nOut + o.nInp;
            %p = get(gcf,'position');
            %set(gcf,'position',p)
            
            if ~exist('clim','var')
                clim = [-1 1];
            end
            
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