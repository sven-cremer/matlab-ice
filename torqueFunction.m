function tau = torqueFunction(robot, t, q, qd)

global xt;  % qt
global Pgain Dgain na;
global lastUpdate updateStep;
global tau;
global data;
global NN_off GC_on

% From main:
% t = qt(:,1);      % 0 -> 10
% q = qt(:,2:7);    % q(t)

% Check time range
if t > xt(end,1)
    t = xt(end,1);
end

delT = t-lastUpdate;
if( delT < updateStep )
    return;   % No update for tau
end

%% Update tau
lastUpdate = t;

if(GC_on)
    tau_g = robot.gravload(q);
else
    tau_g = zeros(1,6);
end

%% Desired position
x_m_   = interp1(xt(:,1), xt(:,2:7), t)';
xd_m_  = zeros(6,1);    % TODO compute
xdd_m_ = zeros(6,1);

%% Get current robot state
J = robot.jacobn(q);
T = robot.fkine(q);
xC  = [transl(T); tr2rpy(T)'];
xdC = J * qd';

%% Compute control force
if(NN_off)
    %--------------------------
    % PD controller
    
    % Desired joint position 
%     q_int = interp1(qt(:,1), qt(:,2:7), t); % interpolated angle time t (from ode45)
%     
%     e = q_int - q;
%     tau = e * diag(Pgain) - qd * diag(Dgain);
    
    fc = diag(Pgain) * (x_m_-xC)  - diag(Dgain) * xdC;
    
else
    %--------------------------
    % Neuroadaptive controller
    
    J = robot.jacobn(q);
    
    % Get current state
    T = robot.fkine(q);
    xC  = [transl(T); tr2rpy(T)'];
    xdC = J * qd';
    
    f_h    = zeros(6,1);
    
%     delT = t-lastUpdate;
%     if( delT < controllerStep )
%         delT = 0;   % No update to weights
%     else
%         lastUpdate = t;
%     end
    
    na = update(na, q', qd', xC, xdC, x_m_, xd_m_, xdd_m_, f_h, delT);
    fc = na.fc;
    
end

%% Compute control torque
tau = (J'*fc)' + tau_g;

% Torque Saturation
tau_max = 100;
tau_min = -100;
idxMax = tau > tau_max;
idxMin = tau < tau_min;
%     if( sum(idxMax(:)) + sum(idxMin(:)) > 0)
%         r1 = tau_max / max( tau(:) );
%         r2 = tau_min / min( tau(:) );
%         if(r1 < r2)
%             tau = tau .* r1;
%         else
%             tau = tau .* r2;
%         end
%     end
tau( idxMax ) = tau_max;
tau( idxMin ) = tau_min;

% Error checking
idxNaN = isnan(tau);
if( sum( idxNaN(:) ) > 0 )
    disp('Error: x contains NaN values inside odeSim()');
    tau(idxNaN) = 0;
end

%% Save results
k = data.idx;

data.t(k) = t;

data.x_      (k,:) = xC';
data.xd_     (k,:) = xdC';
data.q_      (k,:) = q';
data.qd_     (k,:) = qd';

data.x_m_    (k,:) = x_m_';
data.xd_m_   (k,:) = xd_m_';

data.normW_  (k,:) = norm(na.W);
data.normV_  (k,:) = norm(na.V);

data.fl_     (k,:) = na.fl';
data.f_hat_  (k,:) = na.f_hat';
data.fc_     (k,:) = fc';
data.tau_    (k,:) = tau';
data.lambda_ (k,:) = diag(na.lambda)';
data.gamma_  (k,:) = diag(na.gamma)';

data.idx = k + 1;

%% Display NN weights
if(false)
    plotWeights(na)
    pause(0.0001)
end

%% Display progress
str = sprintf('\nTime completed: %.4f\n',t(end));
fprintf(repmat('\b',1,numel(str)));
fprintf(str);


