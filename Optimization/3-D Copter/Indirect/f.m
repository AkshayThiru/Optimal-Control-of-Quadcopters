function [dxdt] = f (x, u) % x: position, velocity, quaternion, angular velocity (body)

    global Q
    
    dxdt = zeros (13,1);
    
    % q1 = x7, q2 = x8, q3 = x9, q4 = x(10)
    % q = quaternion (x(7:10)');
    
    M = [(u(4)+u(3)-u(2)-u(1)) * Q.l/sqrt(2); 
         (u(3)+u(1)-u(2)-u(4)) * Q.l/sqrt(2);
         (u(4)+u(1)-u(2)-u(3)) * Q.M];
    
    dxdt(1:3)   = x(4:6);
    
    dxdt(4)     =  2*x(7)*x(9) + 2*x(8)*x(10);
    dxdt(5)     = -2*x(7)*x(8) + 2*x(9)*x(10);
    dxdt(6)     =  x(7)*x(7)-x(8)*x(8)-x(9)*x(9)+x(10)*x(10);
    dxdt(4:6)   = -Q.g + sum(u)/Q.m * dxdt(4:6);
    % dxdt(4:6)   = -Q.g + 1/Q.m * subsref(compact(q * quaternion(0,0,0,sum(u)) * q'), struct('type','()','subs',{{[2:4]}}))';
    
    dxdt(7)     = 0.5 * (-x(8)*x(11) - x(9)*x(12) - x(10)*x(13));
    dxdt(8)     = 0.5 * ( x(7)*x(11) + x(9)*x(13) - x(10)*x(12));
    dxdt(9)     = 0.5 * ( x(7)*x(12) - x(8)*x(13) + x(10)*x(11));
    dxdt(10)    = 0.5 * ( x(7)*x(13) + x(8)*x(12) - x(9)*x(11));
    % dxdt(7:10)  = 1/2 * compact(q * quaternion([0; x(11:13)]'))';
    
    dxdt(11:13) = Q.I^(-1) * (M - cross(x(11:13),Q.I*x(11:13)));

end