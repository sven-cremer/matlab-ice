function fc = neuroAdaptiveController(q, qd, x, xd, x_m, xd_m, xdd_m, dt)

global input output hidden
global W V
global W_dot V_dot

% NN controller parameters
Kv     = 10*eye(output)  ;
lam    = 20*eye(output)  ;
F      = 100*eye(hidden) ;
G      = 20*eye(input)   ;
kappa  = 0.1             ;
Kz     = 1               ;
Zb     = 100             ;

% Tracking errors
e  = x_m  - x   ;
ed = xd_m - xd  ;
r  = ed  + lam*e ;  % sliding mode error

% Robustifying term
Z = [ W, zeros(size(W, 1), size(V, 2));
      zeros(size(V, 1), size(W, 2)) V ];
v = - Kz*(norm(Z) + Zb)*r;

% Input to NN
y = [ e; ed; x; xd; x_m; xd_m; xdd_m; q; qd ];

% Nonlinear terms
f_hat = W'*sigmoid(V'*y);

% Control force
fc = Kv*r + f_hat - v;

% Expected control force
global Mx Cx Gx fc_exp
% This is the actual function to be approximated
f_act = Mx*( xdd_m + lam*ed ) + Cx*( xd_m + lam*e ) + Gx;
fc_exp = Kv*r + f_act - v;

% Parameter update
if(dt<=0)
    return
end

% W update
W_dot = F*sigmoid(V'*y)*r' - kappa*F*norm(r)*W;

% V update
sigmoidPrime = sigmoid(V'*y)+sigmoid(V'*y)'*sigmoid(V'*y);
V_dot = G*y*(sigmoidPrime'*W*r)' - kappa*G*norm(r)*V;

%V_dot = G*y*((diag(sigmoid(V'*y))*(eye(length(V'*y)) - ...
%        diag(sigmoid(V'*y))))'*W*r)'  - kappa*G*norm(r)*V;

% Update every timestep
W = W + W_dot*dt;
V = V + V_dot*dt;

end