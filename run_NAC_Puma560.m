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
tf = 3;
ts = 0.0001;

t = [t0:ts:tf]';
N = length(t)

% Neuroadpative controller
global na
input  = nCart*10
output = nCart
hidden = 100
na = classNeuroAdaptive(input,hidden,output);
na.PED_on = 1;

% PD controller
global Pgain Dgain 
%Pgain = 1000.*ones(6,1);
%Dgain = 100.*ones(6,1);

Pgain = [100 500 5000 10 10 10];
Dgain = [10 20 50 5 5 5];

%% Reference trajectory
global qt xt

%Tz = p560.fkine(qz);     % L position
%Tr = p560.fkine(qr);     % Vertical
%Ts = p560.fkine(qs);     % Horizontal

%q0 = [0 -20 -20 0 0 0] .* pi/180
%q1 = [0 -23 -23 0 0 0] .* pi/180
%T0 = p560.fkine(q0)
%T1 = p560.fkine(q1)

x0 = [0.4; -0.3; 0.60];
r0 = [0,0,0];

x1 = [0.2; 0.2; 0.60];
r1 = [0,0,0];

T0 = transl(x0)*rpy2tr(r0);
T1 = transl(x1)*rpy2tr(r1);

q0 = p560.ikine(T0);
q1 = p560.ikine(T1);

% Hold, Move, Hold
n1 = find(t > 0.5, 1, 'first');
n2 = find(t > 2.5, 1, 'first');

% Joint trajectory
[q_1, qd_1] = jtraj(q0, q0, n1    );  
[q_2, qd_2] = jtraj(q0, q1, n2-n1 ); 
[q_3, qd_3] = jtraj(q1, q1, N-n2  ); 
q  = [q_1;q_2;q_3];
qd = [qd_1;qd_2;qd_3];
%[q, qd, qdd] = jtraj(q0, q1, N);

qt = [t q];

% Cartesian trajectory
T_1 = ctraj(T0, T0, n1    );  
T_2 = ctraj(T0, T1, n2-n1 );
T_3 = ctraj(T1, T1, N-n2  ); 
TC = cat(3,T_1,T_2);
TC = cat(3,TC, T_3);
%TC = ctraj(T0, T1, N);

xC = transl(TC);
rC = tr2rpy(TC);
x_ref = [xC rC];

xt = [t x_ref];

%% Check
% T = p560.fkine(q);
% plot(x_ref(:,1:3))
% hold on;
% plot(transl(T))

%% Storing data
global data
data = classData(3000,output,nJoints); % TODO actually < N

%% Start simulation
global lastUpdate controllerStep
lastUpdate = 0;
controllerStep = ts;

global tau
tau = zeros(1,nJoints);

tic

[t_sim, q_sim ,qd_sim] = p560.nofriction.fdyn(tf,@torqueFunction,q0,zeros(1,6));

toc

%% Compute Cartesian path
T = p560.fkine(q_sim);
x_sim = [transl(T), tr2rpy(T)];

%% Cartesian error (min)
M = size(x_sim,1);

err_c = zeros(M,1);
for i=1:M
    dif = x_ref - repmat(x_sim(i,:),N,1);   % Difference
    nor = sqrt(sum(abs(dif).^2,2));         % Norms
    err_c(i) = min( nor );                  % Take the value closest to x_ref 
end

figure;
plot(t_sim, err_c)
total_error = sum(err_c)
title(sprintf('Cartesian error norm vs time (total: %.1f)',total_error))

%% Joint error

q_int = interp1(t, q, t_sim );
err_j = sqrt(sum(abs(q_sim - q_int).^2,2));

% err_j = zeros(M,1);
% for i=1:M
%     dif = q - repmat(q_sim(i,:),N,1);   % Difference
%     nor = sqrt(sum(abs(dif).^2,2));         % Norms
%     err_j(i) = min( nor );                  % Take the value closest to x_ref
% end

figure;
plot(t_sim, err_j)
total_error = sum(err_j)
title(sprintf('Joint error norm vs time (total: %.1f)',total_error))

%% Plot results
figure;
set(gcf,'position',[75   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, q_sim(:,i),'-b')
    plot(t, q(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
    suptitle('JOINT POSITION')
end

figure;
set(gcf,'position',[675   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, qd_sim(:,i),'-b')
    plot(t, qd(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
    suptitle('JOINT VELOCITY')
end

%% Cartesian pose
%{
figure;
set(gcf,'position',[75   675   560   840]);
ylab = {'x [m]','y [m]','z [m]','roll [rad]','pitch [rad]','yaw [rad]'};
for i=1:6
    subplot(6,1,i)
    hold on; grid on;
    
    plot(t_sim, x_sim(:,i),'-b')
    plot(t, x_ref(:,i),':r')
    
    xlabel('Time [s]');
    ylabel(ylab{i})
    suptitle('POSE')
end
%}

%figure
%plot(t,norm(q_sim-q))

%%
%plotVariable(data,'q_')
%plotVariable(data,'qd_')

plotVariable(data,'xd_')
plotVariable(data,'tau_')

%%
plotVariable(data,'x_')
plotVariable(data,'x_m_',0,':r')
legend('x','x_m')
suptitle('Cartesian Pose')

return
%% Animate robot
figure; hold on;

simStep = 0.01;
t_int = [ts:simStep:tf]';
q_int   = interp1(t_sim, q_sim, t_int);

plot(t_sim,'-r');
plot(t_int,'-b');

figure
%p560.delay = ts;
% q_sim(500:end,:)
p560.plot(q_int,'trail',':r','delay',simStep)
disp('Done!')
