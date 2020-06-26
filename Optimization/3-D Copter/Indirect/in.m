function [u] = in (x, y) 

    global Q
    
    u = zeros(4,1);
    
    t  = 1/Q.m * [2*(x(7)*x(9)+x(8)*x(10)), 2*(x(9)*x(10)-x(7)*x(8)), (x(7)*x(7)-x(8)*x(8)-x(9)*x(9)+x(10)*x(10))] * y(4:6);
    c1 = -y(11)*Q.l/Q.I(1,1)/sqrt(2)+y(12)*Q.l/Q.I(2,2)/sqrt(2)+y(13)*Q.M/Q.I(3,3);
    c2 = -y(11)*Q.l/Q.I(1,1)/sqrt(2)-y(12)*Q.l/Q.I(2,2)/sqrt(2)-y(13)*Q.M/Q.I(3,3);
    c3 = +y(11)*Q.l/Q.I(1,1)/sqrt(2)+y(12)*Q.l/Q.I(2,2)/sqrt(2)-y(13)*Q.M/Q.I(3,3);
    c4 = +y(11)*Q.l/Q.I(1,1)/sqrt(2)-y(12)*Q.l/Q.I(2,2)/sqrt(2)+y(13)*Q.M/Q.I(3,3);
    
    % Bounded Input, time
%     u(1) = (-sign(t+c1) + 1)*Q.k/2;
%     u(2) = (-sign(t+c2) + 1)*Q.k/2;
%     u(3) = (-sign(t+c3) + 1)*Q.k/2;
%     u(4) = (-sign(t+c4) + 1)*Q.k/2;

    % Unbounded Input, energy
    u(1) = -(t+c1);
    u(2) = -(t+c2);
    u(3) = -(t+c3);
    u(4) = -(t+c4);

end