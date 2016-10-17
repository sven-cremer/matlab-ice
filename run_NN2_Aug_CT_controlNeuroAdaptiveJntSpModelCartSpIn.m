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
t0= 0; tfin= 10;

% NN size
input  = 18;
output = 2;
hidden = 20;

% Discrete model states
global q_m_z  ;
global qd_m_z ;
global qdd_m_z;

% reference Input
global refIn;
inputFlag = 1;

refIn = [ 0.5  1.5 ]';

% Need to do IK to get thetas, refIn is in cart space
q0 = [ 0 0 ]' ;
% arm parameters 
a1= 1 ; a2= 1 ; % arm parameters     

% Pg. 92 Siciliano
c2 = (refIn(1).^2 + refIn(2).^2 - a1^2 - a2^2)/(2*a1*a2);
s2 = sqrt(1 - c2.^2);
% theta2 is deduced
q0(2) = atan2(s2, c2);

k1 = a1 + a2.*c2;
k2 = a2*s2;
% theta1 is deduced
q0(1) = atan2(refIn(2), refIn(1)) - atan2(k2, k1);

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
       input        ; % 15 input
       output       ; % 16 output
       hidden       ; % 17 hidden
       Wini         ; %
       Vini         ; %
       ];

z_mdl = [q_m_z' qd_m_z' qdd_m_z']; % model

xC0 = [ a1*cos(q0(1)) + a2*cos(q0(1) + q0(2))  ;
       a1*sin(q0(1)) + a2*sin(q0(1) + q0(2)) ];
z_sta = [xC0' 0 0 x0(5:end)']; % sampled states
z_t   = 0;

Ts = 0.05;
% tspan = t0:0.0005:tfin;q0(2)
tspan = [0 Ts];
% t = t0:T:tfin;
t = [];
Fsamples = tfin/Ts;
x = [];
tStart = 0;
tEnd = Ts;
tDel = 0;
xDel = x0';
qdd_m = [];
mod_yddA = [];
time = [];

% adaptOn = 0;

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
    [tDel,xDel]= ode45(@robnn2augNeuroAdaptiveJntSpModelCartSpIn, [tStart tEnd], x0);

    tStart = tDel(end);
    tEnd = tEnd + Ts;
    t  = [ t tDel' ];
    x  = [ x; xDel ];
    x0 = xDel(end,:);

    z_t = [ z_t; z_t(end) + Ts ];

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

    z_mdl = [ z_mdl ; q_m_z' qd_m_z' qdd_m_z' ];

    q  = [x0(1) x0(2)]';
    qd = [x0(3) x0(4)]';
        
    a1= 1 ; a2= 1; % arm parameters 
        
    % Robot Jacobian
    J = [ - sin(q(1))*a1 - a2*sin(q(1) + q(2)), - a2*sin(q(1) + q(2))  ;
            cos(q(1))*a1 + a2*cos(q(1) + q(2)),   a2*cos(q(1) + q(2)) ];
            
    xC = [ a1*cos(q(1)) + a2*cos(q(1) + q(2))  ;
           a1*sin(q(1)) + a2*sin(q(1) + q(2)) ];
    xCd= J*qd;
    
    z_sta = [ z_sta ; xC' xCd' x0(5:end) ];

end

save('sim_rel.mat')

%-------------- END CODE ---------------