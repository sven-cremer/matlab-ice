% Simulates the 6DOF Puma 560 robot using Peter Cork's Robotics Toolbox
% Author: Sven Cremer
clear all; close all; clc;
rng(0)
expName = '01';         % Data directory: data_pumaXX/

saveData        = 1;
plotData        = 1;
saveFigures     = 0;
animateRobot    = 1;
plotNNweights   = 0;

NN_on           = 1;     % If 0, then PID is on
GravityComp_on  = 1;     % TODO turn off

%% Setup simulation

startup_rvc     % Launch Robotics Toolbox

mdl_puma560     % Load Puma 560 robot
% mdl_p8        % Puma on an xy base

nJoints = p560.n;
nCart   = 6;

%% Reference trajectory
tic
fprintf('\nGenerating reference trajectory ...\n')
%Tz = p560.fkine(qz);     % L position
%Tr = p560.fkine(qr);     % Vertical
%Ts = p560.fkine(qs);     % Horizontal

%q0 = [0 -20 -20 0 0 0] .* pi/180
%q1 = [0 -23 -23 0 0 0] .* pi/180
%T0 = p560.fkine(q0)
%T1 = p560.fkine(q1)

global traj

traj = classRefTraj(nJoints);

dt  = 0.001;     % Main trajectory step time
tf  = 10;        % Main trajectory execution time
dtS = 2.0;       % Start position hold time
dtF = 4.0;       % Final position hold time

x0 = [0.4; 0.0; 0.60];
x1 = [0.2; 0.2; 0.60];
xs = [-0.3; 0.3; 0.4];
r0 = [0,0,0];
r1 = [8,2,0].*(pi/180);

% Use ctraj method
%{
traj = traj.straightCtraj(p560, x0, r0, x1, r1, dt, tf);
traj = traj.holdEndpoints(dtS,dtF);

traj.plotTraj();
title('ctraj')
%}
% Hold position
% {
GravityComp_on = 0;
%traj.IKopt ='rdn';  % Elbow down
traj = traj.straightCtraj(p560, xs, r0, xs, r0, dt, 0.1);
traj = traj.holdEndpoints(0.9,24);

traj.plotTraj();
title('ctraj')
% }
% Use jtraj method
%{
traj = traj.straightJtraj(p560, x0, r0, x1, r1, dt, tf);
traj = traj.holdEndpoints(dtS,dtF);

traj.plotTraj();
title('jtraj')
%}
%{
% Circular reference trajectory
radius = 0.10;
%rs = [0 -pi/15 0]; % Unstable at lowest point for >|pi/15|?
rs = [0 0 0];

traj = traj.circular(p560, xs, rs, radius, dt, tf);
traj = traj.holdEndpoints(dtS,dtF);

traj.plotTraj();
title('circle')
%}

N = traj.N;
fprintf('Total simulation time: %.1f sec (%d steps for dt=%.4f)\n',traj.tf, traj.N, traj.dt)
toc
%traj.animateTraj(p560); return

%% Neuroadpative controller
global na
input  = nCart*9;
output = nCart;
hidden = 10;
na = classNeuroAdaptive(input,hidden,output)
na.PED_on = 1;
na.RB_on  = 1;
na.NN_on  = 1;
%fprintf(' Inputs: %d\n Hidden: %d\n Outputs: %d\n',input,hidden,output)

%na.Kv  = diag([2,2,2, 0.01,0.01,0.01]);
%na.lam = diag([20,20,20, 0.1,0.1,0.1]);
na.Kv  = diag([2,2,2, 1,1,1]);
na.lam = diag([20,20,20, 10,10,10]);
%na.Kd  = diag([10,10,10, 5,5,5]);
%na.Dd  = diag([2,2,2, 1,1,1]);

%% PD controller
global Pgain Dgain 
%Pgain = 1000.*ones(6,1);
%Dgain = 100.*ones(6,1);
Pgain = [100 500 5000 10 10 10];
Dgain = [10 20 50 5 5 5];

%% Storing data
global data
data = classData(round(0.5*N),output,nJoints); % TODO actually < N

%% Start simulation
global NN_off GC_on
global lastUpdate updateStep
global tau

NN_off  = ~NN_on;
GC_on   = GravityComp_on;
lastUpdate = 0;
updateStep = 0.0; %traj.dt;   % TODO larger?
tau = zeros(1,nJoints);

tic

[t_sim, q_sim ,qd_sim] = p560.nofriction.fdyn(traj.tf,@torqueFunction,traj.q0,zeros(1,nJoints));

fprintf('\n')
toc

fprintf('Data points: %d\n',data.idx);

%% Compute Cartesian path
T = p560.fkine(q_sim);
x_sim = [transl(T), tr2rpy(T)];

%% Cartesian error
x_int = interp1(traj.t, traj.x, t_sim );
err_c = sqrt(sum(abs(x_sim - x_int).^2,2));

err_c_tot = sum(err_c);
fprintf('Total Cartesian error:\t\t%.1f\n',err_c_tot);

% Minimum error (ignoring time)
M = size(x_sim,1);
err_cm = zeros(M,1);
for i=1:M
    dif = traj.x - repmat(x_sim(i,:),N,1);   % Difference
    nor = sqrt(sum(abs(dif).^2,2));         % Norms
    err_cm(i) = min( nor );                  % Take the value closest to x_ref 
end
fprintf('Total Cartesian error (MIN):\t%.1f\n',sum(err_cm));

figure; hold on; grid on;
plot(t_sim, err_c,'-b');
plot(t_sim, err_cm,'--r');
grid on;
title(sprintf('Cartesian Error Norm (total: %.1f)',err_c_tot))

%% Joint error
q_int = interp1(traj.t, traj.q, t_sim );
err_j = sqrt(sum(abs(q_sim - q_int).^2,2));

err_j_tot = sum(err_j);
fprintf('Total joint error:\t\t%.0f\n',err_j_tot);

% Minimum error (ignoring time)
M = size(q_sim,1);
err_jm = zeros(M,1);
for i=1:M
    dif = traj.q - repmat(q_sim(i,:),N,1);   % Difference
    nor = sqrt(sum(abs(dif).^2,2));         % Norms
    err_jm(i) = min( nor );                  % Take the value closest to x_ref 
end
fprintf('Total joint error (MIN):\t%.0f\n',sum(err_jm));

figure; hold on; grid on;
plot(t_sim, err_j,'-b');
plot(t_sim, err_jm,'--r');
grid on;
title(sprintf('Joint Error Norm (total: %.0f)',err_j_tot))

%% Plot results
figure;
set(gcf,'position',[75   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, q_sim(:,i),'-b')
    plot(traj.t, traj.q(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad]',i))
    if(i==1)
        title('Joint Position')
    end
end

figure;
set(gcf,'position',[75   675   560   840]);
for i=1:nJoints
    subplot(nJoints,1,i)
    hold on; grid on;
    plot(t_sim, qd_sim(:,i),'-b')
    plot(traj.t, traj.qd(:,i),':r')
    xlabel('Time [s]');
    ylabel(sprintf('Joint %d [rad/s]',i))
    if(i==1)
        title('Joint Velocity')
    end
end

%plotVariable(data,'q_')
%plotVariable(data,'qd_')

% Cartesian pose
plotVariable(data,'x_')
plotVariable(data,'x_m_',0,':r')
legend('act','ref')

% Cartesian velocity
plotVariable(data,'xd_')
plotVariable(data,'xd_m_',0,':r')
legend('act','ref')

plotVariable(data,'tau_')
%plotVariable(data,'tau_exp',0,':r')
%legend('NN','Actual')

plotVariable(data,'fl_')

plotVariable(data,'f_hat_')
plotVariable(data,'f_act_',0,':r')
plotVariable(data,'fc_',0,'-g')
plotVariable(data,'fc_exp',0,':k')
legend('f_hat', 'expected', 'fc', 'expected')

plotVariable(data,'gamma_')
plotVariable(data,'lambda_')

plotVariable(data,'normW_')
plotVariable(data,'normV_')

figure
plotWeights(na, [-1 1])
colorbar;
figure
plotWeights(na, [-0.01 0.01])
colorbar;

figure
imagesc(na.rbf_mu')
title('RBF mu')

figure;
grid on;
plot(1000.*diff(t_sim))
ylabel('Time step [ms]'); xlabel('ODE iteration number');

%% Save figures and results
fName = 'sim.mat';

% Create folders
dirData = ['data_puma',expName];
dirFigs = [dirData,'/fig'];

if ~exist(dirData, 'dir')
    mkdir(dirData);
end
if ~exist(dirFigs, 'dir')
    mkdir(dirFigs);
end

if(saveData)
    save([dirData,'/',fName],'traj','na','data')
end

if(saveFigures)
    h = findobj('type','figure');
    n = length(h);   
    for i=1:n
        %saveas(figure(j),[dirFigs,'/eps/',fnames{j},int2str(i),'.eps'],'epsc')
        saveas(figure(i),[dirFigs,'/',int2str(i),'.png'],'png')
    end
end

%% Animate robot
if(~animateRobot)
    return
end
%%
% Compute q(t) with a fixed time step
simStep = 0.01;
t_int   = (t_sim(1):simStep:t_sim(end))';
q_int   = interp1(t_sim, q_sim, t_int);

w = 0.75;
W = [-w, w -w w -0.2 1.0];

% Setup plot
figure
p560.plot(q_int(1,:),'trail','-m','workspace',W,'scale',0.7) % 'zoom',1.4,'floorlevel',-0.1

% Draw reference trajectory
hold on;
plot3(traj.x(:,1),traj.x(:,2),traj.x(:,3),':k')

% Animate robot
fprintf('Running animation ... ')
p560.plot(q_int,'delay',0.1*simStep)
fprintf('done!\n')

saveas(gcf,[dirFigs,'/animation.png'],'png')

