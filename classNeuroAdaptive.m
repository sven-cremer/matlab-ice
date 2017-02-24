classdef classNeuroAdaptive
    % Neuroadaptive controller class
    
    properties
        % Public properties
        
        V;  % Inner weights, [nInp x nHid]
        W;  % Outer weights, [nHid x nOut]
        
        % NN controller parameters
        Kv;
        lam;
        F;
        G;
        kappa  = 0.001;
        Kz     = 0.05;
        Zb     = 100;
        
        % Output
        fhat;   % NN output
        fc;     % Control force
        fc_exp; % Expected control force
        
    end
    
    properties (SetAccess = private)
        % Allow read-only. To change use set methods.
        
        nInp;      % Number of inputs
        nHid;     % Size of hidden layer
        nOut;     % Number of outputs
        
    end
    
    methods     % constructor method
        
        function o = classNeuroAdaptive(nInput, nHidden, nOutput)
            
            o.nInp = nInput;
            o.nHid = nHidden;
            o.nOut = nOutput;
            
            % Initialize random weights on the interval [a, b]
            a = -0.1; b=0.1;
            o.W = a + (b-a).*rand( o.nHid, o.nOut );
            o.V = a + (b-a).*rand( o.nInp , o.nHid );
            
            o.Kv     = 10  *eye(o.nOut) ;
            o.lam    = 20  *eye(o.nOut) ;
            o.F      = 100 *eye(o.nHid) ;
            o.G      = 100 *eye(o.nInp) ;
            
            o.fhat   = 0;
            o.fc     = 0;
            o.fc_exp = 0;
            
        end
        
    end
    
    methods    % set/get methods
        
        function o = set.numHidden(o, i)
            validateattributes(i, {'integer'}, ...
                {'positive'});
            o.nHid = i;
            %TODO: resize W, V, etc.
        end
        
    end
    
    methods     % computational methods
        
        function o = update(o, q, qd, x, xd, x_m, xd_m, xdd_m, dt)
            % NN update step
            
            % Tracking errors
            e  = x_m  - x   ;
            ed = xd_m - xd  ;
            r  = ed  + o.lam*e ;  % sliding mode error
            
            % Robustifying term
            Z = [ o.W, zeros(o.nHid, o.nHid)
                  zeros(o.nInp, o.nOut), o.V ];
            v = - o.Kz*(norm(Z) + o.Zb)*r;
            
            % Input to NN
            y = [ e; ed; x; xd; x_m; xd_m; xdd_m; q; qd ];
            
            % Nonlinear terms
            S = sigmoid(o.V'*y);              % Hidden layer output
            o.f_hat = o.W'*S;
            
            % Control force
            o.fc = o.Kv*r + 1.0*(o.f_hat - v);  % TODO flag for NN off
            
            % Expected Control force
            [Mx,Cx,Gx] = robotDynamicsCartesian(q, qd);
            f_act = Mx*( xdd_m + o.lam*ed ) + Cx*( xd_m + o.lam*e ) + Gx;
            o.fc_exp = o.Kv*r + f_act - v;
            
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
             
    end
    
    
end




