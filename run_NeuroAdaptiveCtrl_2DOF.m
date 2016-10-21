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

global tau tau_exp

%--------------------------
% Simulation time
t0 = 0;
tf = 10;
Ts = 0.01;   % Controller time step (smaller = better)

%--------------------------
% Arm parameters
global rMass rLength gravity

%rMass = [1.0;1.0];
rMass = [0.4;0.8];
rLength = [1;1];
gravity = 9.8;

%--------------------------
% NN size
input  = 18;
output = 2;
hidden = 25;

global W V
W = zeros( hidden, output );
V = zeros( input , hidden );

%--------------------------
% Discrete Cartesian model states
global x_m_ xd_m_ xdd_m_

% Reference Input
inputFlag = 1;
x_ref = [ 0.5  1.5 ]';

x_m_    = x_ref;	% [ 0 0 ]'; 
x_m_1   = x_m_ ;	% [ 0 0 ]'; % q0 ;

xd_m_   = [ 0 0 ]' ;
xd_m_1  = [ 0 0 ]' ;

xdd_m_  = [ 0 0 ]' ;
xdd_m_1 = [ 0 0 ]' ;

%--------------------------

xC0 = x_ref;%*0.75
q0 = robotInverseKinematics(xC0)
qd0 = [0 0]';

xC0  = robotForwardKinematics(q0)
xdC0 = [0;0];

x0= [  q0(1)        ; %  1 q1
       q0(2)        ; %  2 q2
       qd0(1)       ; %  3 qd1
       qd0(2)       ]; %  4 qd2

N = round((tf-t0)/Ts);     % Data samples

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

t = []; % ODE simulation time vector
x = []; % ODE simulation states

%--------------------------

q = q0;
qd= qd0;
xC = xC0;
xdC = xdC0;
tau = [0;0];
tau_exp = [0;0];

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
    data.normW   (k,:) = norm(W);
    data.normV   (k,:) = norm(V);
    data.x_m     (k,:) = x_m_';
    data.xd_m    (k,:) = xd_m_';
    data.xdd_m   (k,:) = xdd_m_';
    data.tau     (k,:) = tau';
    data.tau_exp (k,:) = tau_exp';
   
    %--------------------------
    % OUTER LOOP
    % Insert your own model traj generator here RLS/MRAC/RL
    %{
    % Change x_ref every 40 samples
    if mod(k,N/5) == 0
        if(inputFlag > 4)
            inputFlag = 1;
        else
            inputFlag = inputFlag + 1;
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
    %x_m_ = x_ref;
    %}
    % Simple model trajectory
    %x_m_ = ([0.5  1.5].*cos([tStart, tStart]))' ;
    A = 1.5;
    x_m_  =   [0.5;1.5].*cos(A*tStart) ;
    xd_m_ =  -[0.5;1.5].*sin(A*tStart).*A ;
    xdd_m_ = -[0.5;1.5].*cos(A*tStart).*(A^2) ;
    
    %--------------------------
    % INNER LOOP
    
    % Forward kinematics
    xC = robotForwardKinematics(q);
    
    % Analytical Jacobian
    J = robotJacobian(q);
    xdC= J*qd;
    
    % NN controller
    [fc,fc_exp] = neuroAdaptiveController(q, qd, xC, xdC, x_m_, xd_m_, xdd_m_,Ts);

    % Computed torques
    Jtrans = J';
    tau     =  Jtrans*fc;
    tau_exp =  Jtrans*fc_exp;
    
    % Torque saturation
    tau_max = 100;
    % [v_max, i_max] = max(abs(tau));
    % if( v_max > tau_max)
    %     tau = tau.*(tau_max/tau(i_max));
    % end
    
    if( abs(tau(1)) > tau_max)
        tau(1) = tau(1).*(tau_max/abs(tau(1)));
    end
    if( abs(tau(2)) > tau_max)
        tau(2) = tau(2).*(tau_max/abs(tau(2)));
    end
    
    %--------------------------
    % SIMMULATION STEP
    
    [tDel,xDel]= ode45(@robotModel, [tStart tStop], x0);
    
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
    if(mod(k,5)==0)
        clc
        percent_complete = k/N*100;
        fprintf('Percent complete: %.0f\n',percent_complete)
    end
    
end

save('sim1.mat')

%-------------- END CODE ---------------