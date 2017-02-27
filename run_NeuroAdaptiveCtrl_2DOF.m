% Authors: Isura Ranatunga
%          Sven Cremer
% University of Texas at Arlington
% Dept. of Electrical Engineering
% UT Arlington Research Institute
%
% Email address: isura@ieee.org
%                sven.cremer@mavs.uta.edu
%
% Website: isura.me
%          svencremer.com
%
% Created:  04/28/2014 - IR
% Modified: 04/06/2014 - IR
%           10/14/2016 - SC
%
% Inner and outer loop
% MRAC Neuroadaptive Control of planar 2-link RR arm
% Inner-loop: Model Reference Neuroadaptive controller
% Outer-loop: ?

%------------- BEGIN CODE --------------
clear all; close all; clc;

saveData = true;
plotData = true;
saveFigs = false;
plotNN   = false;

expName = '02';

%--------------------------

dirData = ['data_exp',expName];
dirFigs = [dirData,'/fig'];

if ~exist(dirData, 'dir')
    mkdir(dirData);
end
if ~exist(dirFigs, 'dir')
    mkdir(dirFigs);
end

fName = 'sim.mat';

%--------------------------
% Simulation time
t0 = 0;
tf = 10;    % 80
Ts = 0.003;   % Controller time step (smaller = better)

%--------------------------
% Robot (2DOF)
robot = classRobot(2);

robot.mass      = [0.4;0.8];
robot.length    = [1;1];

%--------------------------
% Neuroadpative controller
input  = 18;
output = 2;
hidden = 36;

na = classNeuroAdaptive(input,hidden,output);

% Set NN parameters, for example:

% na.lambda = 0.5 .* eye(2);
% na.gamma  = ...

%--------------------------
% Reference Trajctory - TODO make class
discreteRefTraj = true;
inputFlag = 1;

x_ref = [ 0.5  1.5 ]';

x_m_    = x_ref;	% [ 0 0 ]'; 
x_m_1   = x_m_ ;	% [ 0 0 ]'; % q0 ;

xd_m_   = [ 0 0 ]' ;
xd_m_1  = [ 0 0 ]' ;

xdd_m_  = [ 0 0 ]' ;
xdd_m_1 = [ 0 0 ]' ;

%--------------------------

xC0  = x_ref;%*0.75

q0   = IK(robot,xC0)
qd0  = [0; 0];

xC0  = FK(robot,q0)
xdC0 = [0; 0];

x0= [  q0(1)        ; %  1 q1
       q0(2)        ; %  2 q2
       qd0(1)       ; %  3 qd1
       qd0(2)      ]; %  4 qd2

N = round((tf-t0)/Ts);     % Data samples
fprintf('Sampels: %d\n',N)

data.t       = zeros(N,1);
data.xC      = zeros(N,output);
data.xdC     = zeros(N,output);
data.q       = zeros(N,output);
data.qd      = zeros(N,output);
data.normW   = zeros(N,1);
data.normV   = zeros(N,1);
data.x_m     = zeros(N,output);
data.xd_m    = zeros(N,output);
data.xdd_m   = zeros(N,output);
data.tau     = zeros(N,output);
data.tau_exp = zeros(N,output);     % Expected value
data.lambda  = zeros(N,output*2);
data.gamma   = zeros(N,output*2);
data.fl      = zeros(N,output);
data.fh      = zeros(N,1);

data.x_h     = zeros(N,1);
data.xd_h    = zeros(N,1);

t = []; % ODE simulation time vector
x = []; % ODE simulation states

%--------------------------

q = q0;
qd= qd0;
xC = xC0;
xdC = xdC0;
tau = [0;0];
tau_exp = [0;0];

fh   = 0;
x_h  = 0;   % 1D model
xd_h = 0;

tStart = t0;
tStop = tStart + Ts;

for k=1:N
    
    %--------------------------
    % Store data
    data.t       (k,:) = tStop;
    data.xC      (k,:) = xC';
    data.xdC     (k,:) = xdC';
    data.q       (k,:) = q';
    data.qd      (k,:) = qd';
    data.normW   (k,:) = norm(na.W);
    data.normV   (k,:) = norm(na.V);
    data.x_m     (k,:) = x_m_';
    data.xd_m    (k,:) = xd_m_';
    data.xdd_m   (k,:) = xdd_m_';
    data.tau     (k,:) = tau';
    data.tau_exp (k,:) = tau_exp';
    data.lambda  (k,:) = na.lambda(:);
    data.gamma   (k,:) = na.gamma(:);
    data.fl      (k,:) = na.fl(:);
    data.fh      (k,:) = fh(1);
   
    data.x_h     (k,:) = x_h(end);  % 1-D data
    data.xd_h    (k,:) = xd_h(end);
    
    %--------------------------
    % VITE model (fake human)   <- Assume 1-D motion
    %{
    [x_h, xd_h, fh] = viteEstimator( tStart, tStop, xC(1), 10, 400, 100);
    
    fh     = [fh(end);0];
    x_m_   = [0.15;0];
    xd_m_  = [0;0];
    xdd_m_ = [0;0];
    %}
    %--------------------------
    % OUTER LOOP
    % Insert your own model traj generator here RLS/MRAC/RL
    
    if(discreteRefTraj)
        % Change x_ref every 40 samples
        if( mod(k,round(N/5)) == 0 )    % 40
            inputFlag = inputFlag + 1;
            if(inputFlag > 4)
                inputFlag = 1;
            end
        end
        switch inputFlag
            case 1
                x_ref = [ 0.5  1.5 ]';
            case 2
                x_ref = [ 1 1.5 ]';
            case 3
                x_ref = [ 1  1 ]';
            case 4
                x_ref = [ 0.5  1 ]';
            otherwise
                disp('Case for inputFlag not defined!')
        end
        x_m_   = x_ref;
        xd_m_  = [0;0];
        xdd_m_ = [0;0];
    else
        % Simple model trajectory
        %x_m_ = ([0.5  1.5].*cos([tStart, tStart]))' ;
        A = 1.5;
        x_m_  =   [0.5;1.5].*cos(A*tStart) ;
        xd_m_ =  -[0.5;1.5].*sin(A*tStart).*A ;
        xdd_m_ = -[0.5;1.5].*cos(A*tStart).*(A^2) ;
    end
    
    
    %--------------------------
    % Human Intent Estimation
    
    %[xd,xd_dot] = humanIntentEstimator(x, xdot, fh, dt, xd_prev);
    f_h = [0;0];
    
    %--------------------------
    % INNER-LOOP CONTROLLER 
    
    % Robot update
    robot = updateState(robot, q, qd);
    xC    = robot.x;
    xdC   = robot.xd;
    
    % Controller update
    na = update(na, q, qd, xC, xdC, x_m_, xd_m_, xdd_m_, f_h, Ts,robot);
    fc     = na.fc;
    fc_exp = na.fc_exp;
    
    % Computed torques
    tau     =  robot.Jt*fc;
    tau_exp =  robot.Jt*fc_exp;
    
    % Torque saturation
    tau = torqueSaturation(robot, tau);
    robot.tau = tau;
    
    %--------------------------
    % PLANT - ROBOT SIMMULATION STEP
    
    [tDel,xDel]= ode45(@(t,x)odeSim(t,x,robot), [tStart tStop], x0);
  
    % Check for warnings
    [m,id]=lastwarn();
    if strcmp(id,'MATLAB:nearlySingularMatrix')
        disp(strcat('the following warning occurred:'))
        m
        id
        pause;
    end
    lastwarn('');
    
    tStart = tDel(end);
    tStop = tStart + Ts;
    x0 = xDel(end,:);
    
    t  = [ t tDel' ];
    x  = [ x; xDel ];

    % Backward difference  
%     if( k > 1 )
%         xd_m_  = (x_m_ - x_m_1)/Ts ;
%     end
%     if( k > 2 )
%         xdd_m_ = (xd_m_ - xd_m_1)/Ts ;
%     end

    % Store values
    x_m_1   = x_m_   ; 
    xd_m_1  = xd_m_  ;
    xdd_m_1 = xdd_m_ ;

    % Update robot state
    q  = [x0(1) x0(2)]';
    qd = [x0(3) x0(4)]';
   
    % Display progress
    if(mod(k,40)==0)
        clc
        percent_complete = k/N*100;
        fprintf('Percent complete: %.0f\n',percent_complete)
    end
    
    % Visualize NN weights
    if(plotNN)
        plotWeights(na)
        suptitle(sprintf('t=%.3f, e=%f',tStop,norm(xC-x_m_) ));
        pause(0.001)
    end
    
end

% Compute error
e = data.xC - data.x_m;
fprintf('Error norm: %f\n',norm(e))

% Save data
if(saveData)
    save([dirData,'/',fName])
end

% Plot results
if(plotData)
    if(saveFigs)
        plot_NeuroAdaptiveCtrl_2DOF(dirFigs);
    else
        plot_NeuroAdaptiveCtrl_2DOF();
    end
end

%-------------- END CODE ---------------