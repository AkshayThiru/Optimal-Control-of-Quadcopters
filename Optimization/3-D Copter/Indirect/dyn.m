function [dzdt] = dyn (z) % M initial conditions, 1 for each node; z: [26M,1]

    global M Q
    
    % x = zeros(13,1);
    % y = zeros(13,1);
    % u = zeros(4,1);
    % T = z(end);
    
    % q = quaternion (x(7:10)');
    
    dzdt = zeros(26*M,1);
    
    for i = 0:M-1
        x = z(26*i+1:26*i+13);
        y = z(26*i+14:26*(i+1));
        
        u = in(x,y);
        
        dzdt(26*i+1:26*i+13)   = f(x,u);
        dzdt(26*i+14:26*(i+1)) = H_x(x,y,u);
    end
end

