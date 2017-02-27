function tau = torqueFunction(robot, t, q, qd)

global qt xt;
global Pgain Dgain na;
global counter;

NN_off = 1;

% From main:
% t = qt(:,1);      % 0 -> 10
% q = qt(:,2:7);    % q(t)

% Obtain interpolated angle at this time t (from ode45)
if t > qt(end,1)
    t = qt(end,1);
end

q_int = interp1(qt(:,1), qt(:,2:7), t);
%x_int = interp1(xt(:,1), xt(:,2:7), t);

% Compute torque
if(NN_off)
    %--------------------------
    % PD controller
    e = q_int - q;
    tau = e * diag(Pgain) + qd * diag(Dgain);
else
    %--------------------------
    % Neuroadaptive controller
    T = robot.fkine(q);
    x0 = T(1:3,4);
    %r0 = (tr2rpy(T).*(180/pi))';
    r0 = tr2rpy(T)';
    xC = [x0;r0];
   
    J = robot.jacobn(q);
    xdC = J * qd';
    
    T   = robot.fkine(q_int);
    x0 = T(1:3,4);
    %r0 = (tr2rpy(T).*(180/pi))';
    r0 = tr2rpy(T)';
    x_m_ = [x0;r0];
    
    xd_m_  = zeros(6,1);
    xdd_m_ = zeros(6,1);
    f_h    = zeros(6,1);
    
    Ts = min(qt(:,1) - t );
    
    na = update(na, q', qd', xC, xdC, x_m_, xd_m_, xdd_m_, f_h, Ts);
    fc     = na.fc;
    
    tau    =  (J'*fc)';
    
end

% Display progress
counter = counter + 1;
if(mod(counter,100) == 0 )
    str = sprintf('\rTime completed: %.3f\n',t(end));
    fprintf(repmat('\b',1,numel(str)));
    fprintf(str);
end

