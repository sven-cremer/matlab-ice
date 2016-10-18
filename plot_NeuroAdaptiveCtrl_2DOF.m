
clear all; close all; clc;

load('sim1.mat')

%------------- BEGIN CODE --------------

N = size(data.t,1);

[nSteps, nDataPoints] =  size(x);

%% NN weights
figure;
plot(data.t, data.normW)
title('Norm of outer weights W')
figure;
plot(data.t, data.normV)
title('Norm of inner weights V')
%legend('W','V')

%% Tracking errors between model and real robot
e = data.xC - data.x_m;

figure; hold on;
h = plot(data.t,e);
title('Cartesian Error Plot');
xlabel('Time (s)');
ylabel('Error (m)');
legend('e1','e2');
grid on;

%% Torque
figure;
hold on;
plot(data.t,data.tau(:,1),'b-')
plot(data.t,data.tau(:,2),'r-')
plot(data.t,data.tau_exp(:,1),'g--')
plot(data.t,data.tau_exp(:,2),'k:')
title('Input Force and Actual vs Model Cartesian Positions');
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


%% ODE data
figure;
plot(t',x)
legend('x','y', 'dx','dy');

%% Animation
figure;
hold on;
x_min = min(data.xC(:,1)); x_max = max(data.xC(:,1));
y_min = min(data.xC(:,2)); y_max = max(data.xC(:,2));

b = 0.05;

xlim([(1-b)*x_min (1+b)*x_max]);
ylim([(1-b)*y_min (1+b)*y_max]);
for i=2:N

    plot(data.xC(i,1),data.xC(i,2),'rx')
    plot(data.x_m(i,1),data.x_m(i,2),'bo')
    
    lx = [data.xC(i-1,1), data.xC(i,1)];
    ly = [data.xC(i-1,2), data.xC(i,2)];
    
    lx_m = [data.x_m(i-1,1), data.x_m(i,1)];
    ly_m = [data.x_m(i-1,2), data.x_m(i,2)];
    
    plot(lx_m,ly_m,'b:')
    plot(lx,ly,'r-')
    
    pause(0.01)
end
%return

return

%% -------------------------------

% Cartesian positions
figure; hold on; grid on;
plot(data.t,data.xC(:,1),'b-')
plot(data.t,data.xC(:,2),'r-')
plot(data.t,data.x_m(:,1),'g--')
plot(data.t,data.x_m(:,2),'k:')
title('Cartesian position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x','y', 'x_d','y_d}');



%% -------------------------------
    
% figure;
% subplot(2,1,1)
% plot(x(:,49),x(:,51), 'k', 'LineWidth', 2 )
% title('Van der Pol oscillator based Force Phase plot');
% xlabel('f_{h1}');
% ylabel('fDot_{h1}');
% hold on
% subplot(2,1,2)
% plot(x(:,50),x(:,52), 'k', 'LineWidth', 2 )
% hold off
% title('Van der Pol oscillator based Force Phase plot');
% xlabel('f_{h2}');
% ylabel('fDot_{h2}');

%% -------------------------------
    figure(5)
    
    subplot(2,2,1)
    plot(t, tau_h(:, 1), 'k', 'LineWidth', 2 )
    title('Input Force in X axis');
    xlabel('Time (s)');
    ylabel('Force (N)');
    legend('f_{h1}');
    
    subplot(2,2,3)
    plot(t, x(:,1) , 'k', 'LineWidth', 2 )
    hold on
    plot(t, x(:,45), '--r', 'LineWidth', 2 )
    title('Actual vs Model Cartesian Positions for X axis');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('q_1', 'q_{1m}');
    
    subplot(2,2,2)
    plot(t, tau_h(:, 2), 'k', 'LineWidth', 2 )
    title('Input Force in Y axis');
    xlabel('Time (s)');
    ylabel('Force (N)');
    legend('f_{h2}');
    
    subplot(2,2,4)
    plot(t, x(:,2) , 'k', 'LineWidth', 2 )
    hold on
    plot(t, x(:,46), '--r', 'LineWidth', 2 )
    title('Actual vs Model Cartesian Positions for Y axis');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('q_2','q_{2m}');
    hold off
    
    
    %-------------------------------
     
    
    % FIGURE 6
    figure(6)
    lwidth = 1.5;
    subplot(2,2,1)
    plot(z_t, z_sta(:,1) , '.k', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 1), '--r', 'LineWidth', lwidth )
%     hold on
%     plot(z_t, z_sta(:, 22), 'b', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('x Position');
    xlabel('Time (s)');
    ylabel('x (m)');
    legend('q_1', 'q_{1m}', 'q_{1d}');
    
    subplot(2,2,2)
    plot(z_t, z_sta(:,2) , '.k', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 2), '--r', 'LineWidth', lwidth )
%     hold on
%     plot(z_t, z_sta(:, 23), 'b', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('y Position');
    xlabel('Time (s)');
    ylabel('y (m)');
    legend('q_2','q_{2m}', 'q_{2d}');

    subplot(2,2,3)
    plot(z_t, z_sta(:,3) , '.k', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 3), '--r', 'LineWidth', lwidth )
%     hold on
%     plot(z_t, z_sta(:, 24), 'b', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('x Velocity');
    xlabel('Time (s)');
    ylabel('x Velocity (m/s)');
    legend('qd_1', 'qd_{1m}', 'qd_{1d}');
    
    subplot(2,2,4)
    plot(z_t, z_sta(:,4) , '.k', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 4), '--r', 'LineWidth', lwidth )
%     hold on
%     plot(z_t, z_sta(:, 25), 'b', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('y Velocity');
    xlabel('Time (s)');
    ylabel('y Velocity (m/s)');
    legend('qd_2','qd_{2m}', 'qd_{2d}');
    hold off
    
    
    % FIGURE 7
    figure(7)
    lwidth = 1.5;
    subplot(2,2,1)
    plot(z_t, z_sta(:, 22), '.b', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 1), '--r', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('Joint Angles for Joint 1');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('q_{1d}', 'q_{1m}');
    
    subplot(2,2,2)
    plot(z_t, z_sta(:, 23), '.b', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 2), '--r', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('Joint Angles for Joint 2');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('q_{2d}', 'q_{2m}');

    subplot(2,2,3)
    plot(z_t, z_sta(:, 24), '.b', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 3), '--r', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('Joint Velocity for Joint 1');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('qd_{2d}', 'qd_{2m}');
    
    subplot(2,2,4)
    plot(z_t, z_sta(:, 25), '.b', 'LineWidth', lwidth )
    hold on
    plot(z_t, z_mdl(:, 4), '--r', 'LineWidth', lwidth )
    xlim([0 z_t(end)])
    title('Joint Velocity for Joint 2');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('qd_{2d}', 'qd_{2m}');
    hold off
    
    % FIGURE 8
    figure(8)
    lwidth = 1.5;

    plot(z_sta(:,1)  , z_sta(:,2)  , '.k' , 'LineWidth', lwidth )
    hold on 
    plot(z_mdl(:, 1) , z_mdl(:, 2) , '--r', 'LineWidth', lwidth )
%     hold on
%     plot(z_sta(:, 22), z_sta(:, 23), 'b'  , 'LineWidth', lwidth )
%     xlim([0 z_t(end)])
    title('Cartesian Pose');
    xlabel('x (m)');
    ylabel('y (m)');
    legend('q_1', 'q_{1m}', 'q_{1d}');
         
    
%-------------- END CODE ---------------