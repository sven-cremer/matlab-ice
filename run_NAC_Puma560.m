% Simulates the 6DOF Puma 560 robot using Peter Cork's Robotics Toolbox
% Author: Sven Cremer
clear all; close all; clc;

startup_rvc     % Launch Robotics Toolbox

mdl_puma560     % Load Puma 560 robot

nJoints = p560.n;
%p560.gravity = [0;0;0];

% Simulation time
t0 = 0;
tf = 5;
ts = 0.05;

t = [t0:ts:tf]';
N = length(t);

% Neuroadpative controller
global na
input  = nJoints*9
output = nJoints
hidden = 540
na = classNeuroAdaptive(input,hidden,output);

% PD controller
global Pgain Dgain 
Pgain = [20 100 20 5 5 5];
Dgain = [-5 -10 -2 0 0 0];

% Reference trajectory
global qt xt

Tz = p560.fkine(qz);     % L position
Tr = p560.fkine(qr);     % Vertical
Ts = p560.fkine(qs);     % Horizontal

q0 = [0 -10 -20 0 0 0] .* pi/180
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

global lastUpdate;
lastUpdate = 0;

tic

[T, q ,qd] = p560.nofriction.fdyn(10,@torqueFunction,q0,zeros(1,6));

toc

e = q(end,:) - q1
norm(e)

%% Plot results
h = figure;
set(h,'position',[75   675   560   840]);

for i=1:nJoints
    subplot(nJoints,1,i)
    
    plot(T,q(:,i))
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
end
 
%% Animate robot
figure
p560.plot(q)
