function tau = torqueFunction(robot, t, q, qd)

global traj;
global Pgain Dgain na;
global lastUpdate updateStep;
global tau;
global data;
global NN_off GC_on
global counter;

% From main:
% t = qt(:,1);      % 0 -> 10
% q = qt(:,2:7);    % q(t)

% Check time range
if t > traj.tf
    t = traj.tf;
end

% Note: fdyn uses ode45 which has a variable step size
delT = t-lastUpdate;
if( delT < updateStep )
    return;   % No update for tau
end

%% Update tau
lastUpdate = t;

if(GC_on)
    tau_g = robot.gravload(q);
else
    %tau_g = zeros(1,6);
    tau_g = robot.gravload(q).*exp(-0.2*t^2);   % Zero at ~5sec
end

%% Desired position
x_m_   = interp1(traj.t, traj.x,   t)';
xd_m_  = interp1(traj.t, traj.xd,  t)';
xdd_m_ = interp1(traj.t, traj.xdd, t)';

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
    
    f_h    = zeros(6,1);
    
    tmp = [];
    %{
    % Dynamics - Joint Space
    Mq = robot.inertia(q)';     % TODO transpose?
    Cq = robot.coriolis(q,qd)';
    Gq = robot.gravload(q)';
    %Fq = robot.friction(q);
    
    % Jacobians
    Jdot  = robot.jacob_dot(q,qd);
    Jinv  = pinv(J);
    Jt    = J';
    JtransInv   = pinv(Jt);
    
    % Dynamics - Cartesian space
    tmp.Mx = JtransInv*Mq*Jinv;
    tmp.Cx = JtransInv*( Cq - Mq*Jinv*Jdot )*Jinv;
    tmp.Gx = JtransInv*Gq;  
    %}
    
    na = update(na, q', qd', xC, xdC, x_m_, xd_m_, xdd_m_, f_h, delT, tmp);
    fc = na.fc;
    
    %{
    fc_exp = na.fc_exp;
    tau_exp = (J'*fc_exp)' + tau_g;
    
    tau_exp( tau_exp >  100 ) =  100;
    tau_exp( tau_exp < -100 ) = -100;
    
    data.fc_exp     (data.idx,:) = fc_exp';
    data.tau_exp    (data.idx,:) = tau_exp';
    %}
    
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
data.f_act_  (k,:) = na.f_act';
data.fc_     (k,:) = na.fc';
data.fc_exp  (k,:) = na.fc_exp';
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
if( t < 0.5)
    counter = -1;
else
    if(counter < 0)
        fprintf(' Sim time:  %.4f',t);
        counter = 1;
    else
        if( mod(counter,50) == 0 )
            str = sprintf('%.4f',t);
            fprintf(repmat('\b',1,numel(str)));
            fprintf(str);
        end
        counter = counter + 1;
    end
end


