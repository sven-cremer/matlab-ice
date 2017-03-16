% Animates a 2 DOF robot in the xy-plane with time on the z-axis
% Author: Sven Cremer
clear all; close all; clc;

expName = '02';
fName   = 'sim.mat';
step    = 10;

%------------- BEGIN CODE --------------

load(['data_exp',expName,'/',fName])

m1 = robot.mass(1);
m2 = robot.mass(2);
a1 = robot.length(1);
a2 = robot.length(2);

%% Animation
figure;
hold on;
grid on;
x_min = min(data.xC(:,1)); x_max = max(data.xC(:,1));
y_min = min(data.xC(:,2)); y_max = max(data.xC(:,2));

b = 0.75;
%xlim([(1-b)*x_min (1+b)*x_max]);
%ylim([(1-b)*y_min (1+b)*y_max]);

xlabel('x');ylabel('y');zlabel('z');
z = 0;

M = 30;

for i=2:step:N
    
    clf;
    hold on; grid on;
    %z = i-2;
    
    % Limits
    z0 = i - M;
    if(z0<0)
        z0=1;
    end
    zf = i;
    if(zf<M)
        zf=M;
    end
    %zlim([z0 zf]);   % TODO use buffer instead + plot state function

    % Robot
    %drawRobot(data.q(i,:),data.xC(i,:),data.x_m(i,:),a1,i)
    robot.drawRobot(data.q(i,:),data.xC(i,:),i)
    
    % Path taken
    for k=1:M
        idx = i-k+1;
        if(idx<2)
            break;
        end
        lx = [data.xC(idx-1,1), data.xC(idx,1)];
        ly = [data.xC(idx-1,2), data.xC(idx,2)];
        lz = [idx-1, idx];
        
        lx_m = [data.x_m(idx-1,1), data.x_m(idx,1)];
        ly_m = [data.x_m(idx-1,2), data.x_m(idx,2)];
        
        plot3(lx_m,ly_m,lz,'b-')
        plot3(lx,ly,lz,'r-')
        
        % End effector
        plot3(data.xC(idx,1),data.xC(idx,2),idx,'rx','MarkerSize',6)
        plot3(data.x_m(idx,1),data.x_m(idx,2),idx,'bo','MarkerSize',6)
        
        % Error
        ex = [data.xC(idx,1), data.x_m(idx,1)];
        ey = [data.xC(idx,2), data.x_m(idx,2)];
        ez = [idx, idx];
        plot3(ex,ey,ez,'m:')
        
    end
    
    % Origin
    plot3([0;0],[0;0],[z0;zf],'k:')
    
    
    xlim([-1.2 1.2])
    ylim([-2 2])
    zlim([z0 zf]);
    view([-60 20])
    %{
    CameraPosition: [52.7601 71.9889 725.0694]
         CameraPositionMode: 'manual'
               CameraTarget: [0 0 93]
           CameraTargetMode: 'auto'
             CameraUpVector: [1 0 0]
         CameraUpVectorMode: 'manual'
            CameraViewAngle: 1.1668
        CameraViewAngleMode: 'manual'
                   Clipping: 'on'
              ClippingStyle: '3dbox'
    %}
    %set(gcf,'Position',[538 504 729 551])
    set(gcf,'Position',[86 142 909 937])
    
    %set(gca,'Position',[0.1300 0.1100 0.7750 0.8150])
    set(gca,'CameraPosition',[47.7452 64.8164 (632.6631-M+zf)])
    %set(gca,'CameraTarget',[0.4304 0.5127 (21.7554-M+zf)])
    set(gca,'CameraUpVector',[1 0 0])
    %set(gca,'CameraViewAngle',2.0)
    pbaspect([1 1 3])

    %cameratoolbar('SetCoordSys','x')
    %campos([-7.6676   -2.2547  837.2965])
    %campos([34.3364   15.4267  769.9584])
    pause(0.001)
end