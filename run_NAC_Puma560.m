% Simulates the 6DOF Puma 560 robot using Peter Cork's Robotics Toolbox
% Author: Sven Cremer
clear all; close all; clc;

startup_rvc     % Launch Robotics Toolbox

mdl_puma560     % Load Puma 560 robot
% mdl_p8        % Puma on an xy base
nJoints = p560.n;
nCart   = 6;
%p560.gravity = [0;0;0];

% Simulation time
t0 = 0;
tf = 2;
ts = 0.05;

t = [t0:ts:tf]';
N = length(t);

% Neuroadpative controller
global na
input  = nCart*10
output = nCart
hidden = 15
na = classNeuroAdaptive(input,hidden,output);
na.PED_on = 1;

% PD controller
global Pgain Dgain 
%Pgain = 1000.*ones(6,1);
%Dgain = 100.*ones(6,1);

Pgain = [500 500 500 5 5 5];
Dgain = [20 20 20 1 1 1];

% Reference trajectory
global qt xt

Tz = p560.fkine(qz);     % L position
Tr = p560.fkine(qr);     % Vertical
Ts = p560.fkine(qs);     % Horizontal

q0 = [0 -20 -20 0 0 0] .* pi/180
q1 = [0 -23 -23 0 0 0] .* pi/180
T0 = p560.fkine(q0)
T1 = p560.fkine(q1)

[q, qd, qdd] = jtraj(q0, q1, N);    % Joint

TC = ctraj(T0, T1, N);              % Cartesian
xC = transl(TC);
rC = tr2rpy(TC);
%plot(t, xC);

qt = [t q];
xt = [t xC rC];

%% Start simulation
global counter
counter = 0;

global lastUpdate controllerStep;
lastUpdate = 0;
controllerStep = 0.05;

tic

[t_sim, q_sim ,qd_sim] = p560.nofriction.fdyn(tf,@torqueFunction,q0,zeros(1,6));

toc

e = q_sim(end,:) - q1
norm(e)

%% Plot results
h = figure;
set(h,'position',[75   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, q_sim(:,i),'-b')
    plot(t, q(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
    suptitle('JOINT POSITION')
end

h = figure;
set(h,'position',[675   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, qd_sim(:,i),'-b')
    plot(t, qd(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
    suptitle('JOINT VELOCITY')
end

%figure
%plot(t,norm(q_sim-q))
 
%% Animate robot
figure
p560.plot(q)
