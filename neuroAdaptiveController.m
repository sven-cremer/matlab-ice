function [fc,fc_exp] = neuroAdaptiveController(q, qd, x, xd, x_m, xd_m, xdd_m, fh, dt)

global W V                  % NN weights
global gamma lambda fl      % Prescribed Error dynamics

input  = size(V,1);
hidden = size(V,2);
output = size(W,2);

% NN controller parameters
Kv     = 10*eye(output)  ;
%lambda = 20*eye(output)  ;
F      = 100*eye(hidden) ;
G      = 100*eye(input)   ;
kappa  = 0.001             ;
Kz     = 0.05            ;
Zb     = 100              ;

% Prescribed Error dynamics
%fh = [0.0;0.0];                    % TODO measure
Kd = 20.0*eye(output);
Dd = 10.0*eye(output);
gamma = Dd - lambda;
lambda_dot = Kd - gamma*lambda;
lambda = lambda + lambda_dot*dt;    % New lambda
% if(max(lambda(:)) > 25)
%     lambda = 25*eye(output);
% end
% gamma  = Dd - lambda;               % New gamma
fl_dot = fh - gamma*fl;
fl = fl + fl_dot*dt;                % New fl

% Tracking errors
e  = x_m  - x   ;
ed = xd_m - xd  ;
r  = ed  + lambda*e - fl ;          % Sliding mode error

% Robustifying term
Z = [ W, zeros(size(W, 1), size(V, 2));
      zeros(size(V, 1), size(W, 2)) V ];
v = - Kz*(norm(Z) + Zb)*r;

% Input to NN
y = [ e; ed; x; xd; x_m; xd_m; xdd_m; q; qd ];

% Nonlinear terms
S = sigmoid(V'*y);              % Hidden layer output
f_hat = W'*S;

% Control force
fc = Kv*r + 1.0*(f_hat - v);    % TODO flag for NN off

% Expected Control force
[Mx,Cx,Gx] = robotDynamicsCartesian(q, qd);
f_act = Mx*( xdd_m + lambda*ed ) + Cx*( xd_m + lambda*e ) + Gx;
fc_exp = Kv*r + f_act - v;

% Parameter update
if(dt<=0)
    return
end

% W update
W_dot = F*S*r' - kappa*F*norm(r)*W;

% V update
sigmoidPrime = diag(S)+diag(S)*diag(S);                 % diag(S)*(I - diag(S)
V_dot = G*y*(sigmoidPrime'*W*r)' - kappa*G*norm(r)*V;

% V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) - diag(sigmoid(V'*y))))'*W*r)' ...
%         - kappa*G*norm(r)*V;

% Update every timestep
W = W + W_dot*dt;
V = V + V_dot*dt;

end