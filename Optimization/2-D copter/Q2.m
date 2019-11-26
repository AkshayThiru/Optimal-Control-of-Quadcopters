classdef Q2
    
    properties (Constant)
        m = 0.1;
        g = 9.81;
        l = 0.1;
        k = 2; % upper bound on magnitide of thrust
        I = Q2.m*(Q2.l)^2/3;
    end
    
    methods (Static)
        
        function [ds] = state (~, s) % s(1:6) = x, s(7:12) = l
            ds = zeros(12,1);
            u = zeros(2,1);
            
            u(1) = -Q2.k * sign(-s(8)*sin(s(5))/Q2.m + s(10)*cos(s(5))/Q2.m - s(12)*Q2.l/Q2.I); % | - minimum time, bounded input
            u(2) = -Q2.k * sign(-s(8)*sin(s(5))/Q2.m + s(10)*cos(s(5))/Q2.m + s(12)*Q2.l/Q2.I); % |
            % u(1) = s(8)*sin(s(5))/Q2.m - s(10)*cos(s(5))/Q2.m + s(12)*Q2.l/Q2.I; % | - minimum energy, unbounded input
            % u(2) = s(8)*sin(s(5))/Q2.m - s(10)*cos(s(5))/Q2.m - s(12)*Q2.l/Q2.I; % |

            ds(1) = s(2); % x1
            ds(2) = -(u(1)+u(2))*sin(s(5))/Q2.m; % x2
            ds(3) = s(4); % y1
            ds(4) = -Q2.g + (u(1)+u(2))*cos(s(5))/Q2.m; % y2
            ds(5) = s(6); % theta1
            ds(6) = (u(2)-u(1))*Q2.l/Q2.I; % theta2
            
            ds(7) = 0;
            ds(8) = -s(7);
            ds(9) = 0;
            ds(10) = -s(9);
            ds(11) = s(8)*(u(1)+u(2))*cos(s(5))/Q2.m + s(10)*(u(1)+u(2))*sin(s(5))/Q2.m;
            ds(12) = -s(11);
        end
        
        function err = model (l0)
            global xf x0;

            si = [x0; l0(1:6)];
            opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

            [~, s] = ode45 (@(t,s) Q2.state(t,s), [0 l0(7)], si, opts);
            err = norm(xf- s(end,1:6)',2);

        end
        
    end
    
end

