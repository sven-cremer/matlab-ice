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

% Simulation time
t0 = 0;
tf = 10;
Ts = 0.05;   % Step

% Arm parameters
global mass length gravity

mass = [1;1];
length = [1;1];
gravity = 9.8;

% NN size
global input output hidden
input  = 18;
output = 2;
hidden = 20;

% Discrete model states
global q_m_z  ;
global qd_m_z ;
global qdd_m_z;

% -----------------------------------------------------
% reference Input
inputFlag = 1;

refIn = [ 0.5  1.5 ]';

q0 = robotInverseKinematics(refIn)

%--------------------------
q_m_z     = refIn ; % [ 0 0 ]'; 
q_m_z_1   = q_m_z ; % [ 0 0 ]'; % q0 ;
q_m_z_2   = q_m_z ; % [ 0 0 ]'; % q0 ;

qd_m_z    = [ 0 0 ]' ;
qd_m_z_1  = [ 0 0 ]' ;
qd_m_z_2  = [ 0 0 ]' ;

qdd_m_z   = [ 0 0 ]' ;
qdd_m_z_1 = [ 0 0 ]' ;
qdd_m_z_2 = [ 0 0 ]' ;

Wini = zeros( hidden*output, 1 );
Vini = zeros( input*hidden , 1 );

x0= [  q0(1)        ; %  1 q1
       q0(2)        ; %  2 q2
       0            ; %  3 qd1
       0            ; %  4 qd2
       Wini         ; %
       Vini         ; %
       ];

z_mdl = [q_m_z' qd_m_z' qdd_m_z']; % model

xC0  = robotForwardKinematics(q0);
xdC0 = [0;0]

% tspan = t0:0.0005:tfin;q0(2)
%tspan = [0 Ts];
% t = t0:T:tfin;
% tDel = 0;
% xDel = x0';
% qdd_m = [];
% mod_yddA = [];
% time = [];

% adaptOn = 0;
Fsamples = (tf-t0)/Ts;
nDataPoints = output*hidden+hidden*input+2*output;

z_sta = zeros(Fsamples,nDataPoints);
z_t = zeros(Fsamples,1);

t = [];
x = [];

z_sta(1,:) = [xC0' xdC0' x0(5:end)']; % sampled states
z_t(1,1)   = 0;

tStart = t0;
tStop = tStart + Ts;

for k=1:Fsamples
    
    % Input q_r
    if mod(k,Fsamples/5) == 0
        %inputFlag = xor(inputFlag, 1);
        inputFlag = inputFlag + 1;
    end
    
    if inputFlag == 1
        refIn = [ 0.5  1.5 ]';
    end
    
    if inputFlag == 2
        refIn = [ 1 1.5 ]';
    end
    
    if inputFlag == 3
        refIn = [ 1  1 ]';
    end
    
    if inputFlag == 4
        refIn = [ 0.5  1 ]';
        inputFlag = 0;
    end
    
%     if progressFlag == 1
        clc
        percent_complete = k/Fsamples*100;
        fprintf('Percent complete: %.0f\n',percent_complete)
%     end


    % OUTER LOOP
    % Simple model trajectory
    % Insert your own model traj generator here RLS/MRAC/RL
    q_m_z       = ([0.5  1.5].*cos([tStart, tStart]))' ;
    
    %%%

    % INNER LOOP
    [tDel,xDel]= ode45(@robnn2augNeuroAdaptiveJntSpModelCartSpIn, [tStart tStop], x0);

    tStart = tDel(end);
    tStop = tStart + Ts;
    x0 = xDel(end,:);
    
    t  = [ t tDel' ];
    x  = [ x; xDel ];
    z_t(k,:) = tStop ;

    % Backward difference
    
    if( k > 1 )
        qd_m_z  = (q_m_z - q_m_z_1)/Ts   ;
    end
    
    if( k > 2 )
        qdd_m_z = (qd_m_z - qd_m_z_1)/Ts ;
    end

    q_m_z_1   = q_m_z    ;
    q_m_z_2   = q_m_z_1  ;

    qd_m_z_1  = qd_m_z   ;
    qd_m_z_2  = qd_m_z_1 ;

    qdd_m_z_1 = qdd_m_z  ;
    qdd_m_z_2 = qdd_m_z_1;

    z_mdl(k,:) = [ q_m_z' qd_m_z' qdd_m_z' ];

    % Robot states
    q  = [x0(1) x0(2)]';
    qd = [x0(3) x0(4)]';
    
    % Analytical Jacobian
    J = robotJacobian(q);
    
    % Forward kinematics
    xC = robotForwardKinematics(q);
    
    xCd= J*qd;
    
    z_sta(k,:) = [xC' xCd' x0(5:end) ];

end

save('sim1.mat')

%-------------- END CODE ---------------