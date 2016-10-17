% Author:  Isura Ranatunga
% University of Texas at Arlington
% Dept. of Electrical Engineering
% UT Arlington Research Institute
%
% email address: isura@ieee.org
% Website: isura.me
% Created:  04/28/2014
% Modified: 04/06/2014
%           10/14/2016 - Sven Cremer
%
% Inner and outer loop
% MRAC Neuroadaptive Control of planar 2 link RR arm
% Inner-loop Model Reference Neuroadaptive controller
% Outer-loop ??

%------------- BEGIN CODE --------------

clear all; close all; clc;

global tau

%--------------------------
% Simulation time
t0 = 0;
tf = 10;
Ts = 0.05;   % Step

%--------------------------
% Arm parameters
global mass length gravity

mass = [1;1];
length = [1;1];
gravity = 9.8;

%--------------------------
% NN size
global input output hidden
input  = 18;
output = 2;
hidden = 20;

global W V
global W_dot V_dot
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
x_m_2   = x_m_ ;	% [ 0 0 ]'; % q0 ;

xd_m_   = [ 0 0 ]' ;
xd_m_1  = [ 0 0 ]' ;
xd_m_2  = [ 0 0 ]' ;

xdd_m_  = [ 0 0 ]' ;
xdd_m_1 = [ 0 0 ]' ;
xdd_m_2 = [ 0 0 ]' ;

%--------------------------

q0 = robotInverseKinematics(x_ref);
qd0 = [0 0]';

xC0  = robotForwardKinematics(q0);
xdC0 = [0;0];

x0= [  q0(1)        ; %  1 q1
       q0(2)        ; %  2 q2
       qd0(1)           ; %  3 qd1
       qd0(2)           ]; %  4 qd2
%        Wini         ; %
%        Vini         ; %
%        ];

N = (tf-t0)/Ts;

data.t      = zeros(N,1);
data.xC     = zeros(N,output);
data.xdC    = zeros(N,output);
data.q      = zeros(N,output);
data.qd     = zeros(N,output);
data.normW  = zeros(N,1);
data.normV  = zeros(N,1);
data.x_m    = zeros(N,output);
data.xd_m   = zeros(N,output);
data.xdd_m  = zeros(N,output);
data.tau    = zeros(N,output);

% z_sta = zeros(N,nDataPoints);    % Sampled states
% z_mdl = zeros(N,3*output);       % Model states
% z_t   = zeros(N,1);              % Time vector

t = []; % ODE simulation time vector
x = []; % ODE simulation states

%--------------------------

% z_sta(1,:) = [xC0' xdC0' q0' qd0' norm(W) norm(V)];
% z_mdl(1,:) = [x_m_' xd_m_' xdd_m_'];
% z_t(1,1)   = 0;

q = q0;
qd= qd0;
xC = xC0;
xdC = xdC0;
tau = [0;0];

tStart = t0;
tStop = tStart + Ts;

for k=1:N
    
    %--------------------------
    % Store data
    data.t      (k,:) = tStop;
    data.xC     (k,:) = xC';
    data.xdC    (k,:) = xdC';
    data.q      (k,:) = q';
    data.qd     (k,:) = qd';
    data.normW  (k,:) = norm(W);
    data.normV  (k,:) = norm(V);
    data.x_m    (k,:) = x_m_';
    data.xd_m   (k,:) = xd_m_';
    data.xdd_m  (k,:) = xdd_m_';
    data.tau    (k,:) = tau';
    
%         z_mdl(k,:) = [ x_m_' xd_m_' xdd_m_' ];
%         z_sta(k,:) = [xC' xCd' q' qd' norm(W) norm(V)];
   
    %--------------------------
    % OUTER LOOP
    % Insert your own model traj generator here RLS/MRAC/RL
    
    % Change x_ref every 40 samples
    if mod(k,N/5) == 0
        inputFlag = inputFlag + 1;
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
            inputFlag = 1;
        otherwise
            disp('Case for inputFlag not defined!')
    end
    
    % Simple model trajectory
    %x_m_ = ([0.5  1.5].*cos([tStart, tStart]))' ;
    x_m_ = [0.5;1.5].*cos(tStart) ;
    %x_m_ = x_ref;
    
    %--------------------------
    % INNER LOOP
    [tDel,xDel]= ode45(@robnn2augNeuroAdaptiveJntSpModelCartSpIn, [tStart tStop], x0);

    W = W + W_dot*Ts;
    V = V + V_dot*Ts;
    
    tStart = tDel(end);
    tStop = tStart + Ts;
    x0 = xDel(end,:);
    
    t  = [ t tDel' ];
    x  = [ x; xDel ];

    % Backward difference  
    if( k > 1 )
        xd_m_  = (x_m_ - x_m_1)/Ts ;
    end
    
    if( k > 2 )
        xdd_m_ = (xd_m_ - xd_m_1)/Ts ;
    end

    % Store values
    x_m_1   = x_m_   ; 
    x_m_2   = x_m_1  ;  

    xd_m_1  = xd_m_  ;
    xd_m_2  = xd_m_1 ;

    xdd_m_1 = xdd_m_  ;
    xdd_m_2 = xdd_m_1;

    % Robot states
    q  = [x0(1) x0(2)]';
    qd = [x0(3) x0(4)]';
    
    % Analytical Jacobian
    J = robotJacobian(q);
    
    % Forward kinematics
    xC = robotForwardKinematics(q);
    
    xCd= J*qd;
   
    % Display progress
    if(mod(k,5)==0)
        clc
        percent_complete = k/N*100;
        fprintf('Percent complete: %.0f\n',percent_complete)
    end
    
end

save('sim1.mat')

%-------------- END CODE ---------------