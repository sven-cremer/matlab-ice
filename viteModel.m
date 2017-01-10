function ydot = viteModel( t, y )

    global x_goal
    %global alpha

%     N = length(y)
    
    x  = y(1);
    xd = y(2);
    alpha = y(3);
    
    xdd = alpha.*(-xd + 4.*(x_goal - x));
    
    ydot = [xd; xdd; alpha];
    
end