function plot_NeuroAdaptiveCtrl_2DOF(figDir,dataDir)
% Plots results from run_NeuorAdptiveCtrl_2DOF.m
% Sven Cremer, 2017

% Check if figures should be saved
if ~exist('figDir','var') || isempty(figDir)
    saveFigs = false;
else
    saveFigs = true;
    fprintf('Saving figures to: %s',figDir);
    if ~exist(figDir, 'dir')
        mkdir(figDir);
    end
end

% Check if data should be loaded from file or previous workspace
if exist('dataDir','var') && ~isempty(dataDir)
    fprintf('Loading data from: %s',dataDir);
    load(dataDir);
else
    data = evalin('caller','data');
    t = evalin('caller','t');
    x = evalin('caller','x');
end

%% Human VITE model
figure
plot(data.t,data.fh)
title('Human force')

figure
plot(data.t,data.x_h)
title('Human position')

figure
plot(data.t,data.xd_h)
title('Human velocity')

%% NN weights
figure;
plot(data.t, data.normW)
title('Norm of outer weights W')
figure;
plot(data.t, data.normV)
title('Norm of inner weights V')
%legend('W','V')

%% Force values
figure;
plot(data.t, data.fl)
title('Fl values')

%% Gamma & Lambda
figure;
plot(data.t, data.gamma)
title('Gamma values')
figure;
plot(data.t, data.lambda)
title('Lambda values')

%% Tracking errors between model and real robot
e = data.xC - data.x_m;

error = norm(e)

figure; hold on;
h = plot(data.t,e);
title('Cartesian Error Plot');
xlabel('Time (s)');
ylabel('Error (m)');
legend('e1','e2');
grid on;

%% Torque
figure;
hold on; grid on;
plot(data.t,data.tau(:,1),'b-')
plot(data.t,data.tau(:,2),'r-')
plot(data.t,data.tau_exp(:,1),'g--')
plot(data.t,data.tau_exp(:,2),'k:')
title('Control torque');
xlabel('Time (s)');
ylabel('Position (m)');
legend('NN \tau_{1}','NN \tau_{2}','Actual \tau_{1}','Actual \tau_{2}');

b = 0.5;
y_max = (1+b)*max(max(data.tau));
y_min = (1+b)*min(min(data.tau));
ylim([y_min y_max])

%% Joint positions 
figure; hold on; grid on;
plot(data.t,data.q(:,1),'b-')
plot(data.t,data.q(:,2),'r-')
plot(data.t,data.qd(:,1),'g--')
plot(data.t,data.qd(:,2),'k:')
title('Joint positions');
xlabel('Time (s)');
ylabel('Position (m)');
legend('q_1','q_2', 'qdot_{1}','qdot_{2}');

% figure; hold on;
% plot(data.t,data.qd(:,1),'b-')
% plot(data.t,data.qd(:,2),'r-')
% title('Joint positions');
% xlabel('Time (s)');
% ylabel('Position (m)');
% legend('qdot_{1}','qdot_{2}');

%% Cartesian positions
figure; hold on; grid on;
plot(data.t,data.xC(:,1),'b-')
plot(data.t,data.xC(:,2),'r-')
plot(data.t,data.x_m(:,1),'g--')
plot(data.t,data.x_m(:,2),'k:')
title('Cartesian position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x','y', 'x_r','y_r');

%% Cartesian velocity
figure; hold on; grid on;
plot(data.t,data.xdC(:,1),'b-')
plot(data.t,data.xdC(:,2),'r-')
plot(data.t,data.xd_m(:,1),'g--')
plot(data.t,data.xd_m(:,2),'k:')
title('Cartesian velocity');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x_d','y_d', 'x_r','y_r');

%% Cartesian 2D plot
figure; hold on; grid on;
plot(data.x_m(:,1),data.x_m(:,2),'rx-')
plot(data.xC(:,1),data.xC(:,2),'b.-')
title('Cartesian 2D plot');
xlabel('Time (s)');
ylabel('Position (m)');
legend('ref', 'actual');

%% ODE data
figure;
plot(t',x)
legend('x','y', 'dx','dy');
title('ODE simulation');

%% Save figures
if(saveFigs)
    h =  findobj('type','figure');
    n = length(h);
    
    for i=1:n
        %saveas(figure(j),[figDir,'/eps/',fnames{j},int2str(i),'.eps'],'epsc')
        saveas(figure(i),[figDir,'/',int2str(i),'.png'],'png')
    end
end

%%
end