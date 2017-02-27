function tau = torqueFunction(robot, t, q, qd)

global qt xt;
global Pgain Dgain na;
global counter;
global lastUpdate;

NN_off = 0;

% From main:
% t = qt(:,1);      % 0 -> 10
% q = qt(:,2:7);    % q(t)

% Check time range
if t > qt(end,1)
    t = qt(end,1);
end

% Compute torque
if(NN_off)
    %--------------------------
    % PD controller
    
    % Desired joint position 
    q_int = interp1(qt(:,1), qt(:,2:7), t); % interpolated angle time t (from ode45)
    
    e = q_int - q;
    tau = e * diag(Pgain) + qd * diag(Dgain);
else
    %--------------------------
    % Neuroadaptive controller
    
    J = robot.jacobn(q);
    
    % Get current state
    T = robot.fkine(q);
    xC  = [transl(T); tr2rpy(T)'];
    xdC = J * qd';
    
    % Desired position
    x_m_   = interp1(xt(:,1), xt(:,2:7), t)';
    xd_m_  = zeros(6,1);    % TODO compute
    xdd_m_ = zeros(6,1);
    
    f_h    = zeros(6,1);
    
    delT = t-lastUpdate;
    
    na = update(na, q', qd', xC, xdC, x_m_, xd_m_, xdd_m_, f_h, delT);
    fc     = na.fc;
    
    tau    =  (J'*fc)';
    
    % Torque Saturation
    tau_max = 10;
    tau_min = -10;
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
    
    lastUpdate = t;
end

% Display progress
counter = counter + 1;
if(mod(counter,100) == 0 )
    str = sprintf('\rTime completed: %.3f\n',t(end));
    fprintf(repmat('\b',1,numel(str)));
    fprintf(str);
end

