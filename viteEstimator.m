function [xd, xd_dot, fh] = viteEstimator( tStart, tStop, x0, alpha, Kh, Dh)

if(length(x0)>1)
    disp('Warning: x0 is too large')
end

global x_goal
x_goal = 0.15;

% global alpha
% alpha = 10.0;

x0 = [x0;0;alpha];

% VITE model
[t,x]= ode45(@viteModel, [tStart tStop], x0);

xd      = x(:,1);
xd_dot  = x(:,2);

% Human force
fh = Kh.*(x_goal-xd) - Dh.*xd_dot;
