% Author: Isura Ranatunga,
% University of Texas at Arlington
% Dept. of Electrical Engineering
% UT Arlington Research Institute
%
% email address: isura@ieee.org
% Website: isura.me
% Created: 01/26/2013
% Modified: 04/06/2014
%
% Output of planar 2 link RR arm control
% using 2 layer Augmented Neural Network
% Cartesian Space Model Joint Space Input (Torque)

%------------- BEGIN CODE --------------

% file robout2layerJntSpModelJntSpIn.m
function [tau_h,e]= robout2layerJntSpModelJntSpIn( t, x, z_mdl, z_sta, z_t )
global input output hidden

[nSteps, nData] =  size(x);

% Force input
    tau_h = [ x(:, 9) x(:, 10) ];
   
% tracking errors between model and real robot
    e = x(:,1:2) - x(:, 5: 6) ;

% NN weights
   nnStart = 5;
   endW   = (nnStart + hidden*output-1);
   startV = endW + 1;
   endV   = startV + input*hidden - 1;
   nnWeightNorms = zeros(nSteps,2);
   for i=1:nSteps   
       W      = reshape(x(i,nnStart:endW), output, hidden)';
       V      = reshape(x(i,startV:endV), input , hidden) ;
    
       nnWeightNorms(i,1) = norm(W);
       nnWeightNorms(i,2) = norm(V);
   end
   
   plot(nnWeightNorms)
    
    % Animation
    figure(9)
    hold on;
    xlim([-0.6 0.6]);
    ylim([-2 2]);
    
    x1 = z_sta(:,1);
    x2 = z_sta(:,2);
    for i=1:length(x1)
        
        plot(x1(i),x2(i),'rx')  
        pause(0.01)
    end
    
    %return
    
    
    %-------------------------------
    figure(1)
    h = plot(t,e);
    title('Cartesian Error Plot');
    xlabel('Time (s)');
    ylabel('Error (m)');
    legend('e1','e2');
    
    figure(2)
    plot(t,[tau_h x(:,1) x(:,2) x(:,45) x(:,46)])
    title('Input Force and Actual vs Model Cartesian Positions');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('f_{h1}','f_{h2}','q_1','q_2', 'q_{1m}','q_{2m}');
    
    figure(3)
    plot(t,[tau_h x(:,1)+5 x(:,2)+10 x(:,45)+5 x(:,46)+10])
    title('Input Force and Actual vs Model Cartesian Positions with Offsets');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('f_{h1}','f_{h2}','q_1','q_2', 'q_{1m}','q_{2m}');
    
    figure(4)
    subplot(2,1,1)
    plot(x(:,49),x(:,51), 'k', 'LineWidth', 2 )
    title('Van der Pol oscillator based Force Phase plot');
    xlabel('f_{h1}');
    ylabel('fDot_{h1}');
    hold on
    subplot(2,1,2)
    plot(x(:,50),x(:,52), 'k', 'LineWidth', 2 )
    hold off
    title('Van der Pol oscillator based Force Phase plot');
    xlabel('f_{h2}');
    ylabel('fDot_{h2}');
    
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