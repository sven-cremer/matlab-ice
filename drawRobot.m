function drawRobot(q,x,x_m,a1,z)

z2 = z*ones(1,2);

    % Robot
    lx01 = [0, a1*cos(q(1))];
    ly01 = [0, a1*sin(q(1))];
    lx12 = [a1*cos(q(1)),x(1)];
    ly12 = [a1*sin(q(1)),x(2)];
    plot3(lx01,ly01,z2,'g-','LineWidth',2)
    plot3(lx12,ly12,z2,'g-','LineWidth',2)
    plot3([lx01,lx12], [ly01,ly12], [z2, z2],'g.','MarkerSize',16)
    
%     % End effector
%     plot3(x(1),x(2),z2,'rx','MarkerSize',8)
%     plot3(x_m(1),x_m(2),z2,'bo','MarkerSize',8)
    
    % Path taken
    %lx = [data.xC(i-1,1), data.xC(i,1)];
    %ly = [data.xC(i-1,2), data.xC(i,2)];
    
    %lx_m = [data.x_m(i-1,1), data.x_m(i,1)];
    %ly_m = [data.x_m(i-1,2), data.x_m(i,2)];
    
    %plot3(lx_m,ly_m,z2,'b:')
    %plot3(lx,ly,z2,'r:')


end