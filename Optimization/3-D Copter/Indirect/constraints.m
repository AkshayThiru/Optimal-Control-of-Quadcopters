function [c, ceq] = constraints (v) % v: variables (26*(M-1)+14)

    global M Init

    c = [];
    
    % ceq = zeros(26*(M-1)+1,1);
    ceq = zeros(26*(M-1)+14+(M-1),1);
    
     % Initial conditions for all times
    z0 = [Init.xi; v(1:end-1)]; % size: 26*M
    
    % Final states
    [~,z] = ode45(@(t,z) dyn(z), [0, v(end)/M], z0);
    zf = z(end,1:end)'; % size: 26*M: for t = tf/M
    
%     figure(1)
%     hold on
%     title('Minimum Energy, Unbounded Input, M = 4','Interpreter','latex');
%     xlabel('x (m)','Interpreter','latex');
%     ylabel('y (m)','Interpreter','latex');
%     zlabel('z (m)','Interpreter','latex');
%     for i = 0:M-1
%         plot3(z(:,26*i+1),z(:,26*i+2),z(:,26*i+3),'-b');
%     end
%     plot3(0,0,0,'ok');
%     plot3(0,0,1,'ok');
%     grid on
%     hold off
    
    % Constraints for t = 0 - (M-1)*tf/M
    if (M > 1)
        ceq(1:26*(M-1)) = z0(27:end) - zf(1:26*(M-1));
    end
    
    if (M > 1)
        for i = 0:(M-2)
            ceq(26*(M-1)+14+i+1) = (v(26*i+20:26*i+23)'*v(26*i+20:26*i+23)) - 1;
        end
    end
    
    % Constraints for t = tf
    
    % Final position, velocity constraints: 6
    %ceq(26*(M-1)+1:26*(M-1)+6) = zf(26*(M-1)+14:26*(M-1)+19) - 2*Init.r*(zf(27*(M-1)+1:27*(M-1)+6) - Init.xd(1:6));
    ceq(26*(M-1)+1:26*(M-1)+6) = zf(26*(M-1)+1:26*(M-1)+6) - Init.xd(1:6);
    
    % Final orientation constraints: 4
    % ceq(26*(M-1)+7:26*(M-1)+10) = zf(27*(M-1)+20:27*(M-1)+23) - 2*Init.r*(zf(27*(M-1)+7:27*(M-1)+10) - Init.xd(7:10));
    ceq(26*(M-1)+7:26*(M-1)+10) = zf(26*(M-1)+7:26*(M-1)+10) - Init.xd(7:10);
    
    % Final angular velocity constraints: 3
    % ceq(26*(M-1)+11:26*(M-1)+13) = zf(27*(M-1)+24:27*(M-1)+26) - 2*Init.r*(zf(27*(M-1)+11:27*(M-1)+13) - Init.xd(11:13));
    ceq(26*(M-1)+11:26*(M-1)+13) = zf(26*(M-1)+11:26*(M-1)+13) - Init.xd(11:13);
    
    % Final Lagrange multiplier constraint: 1
    % ceq(end) = zf(end) - 2*z0(end);
    yf = zf(26*(M-1)+14:end);
    xf = zf(26*(M-1)+1:26*(M-1)+13);
    uf = in(xf,yf);
    L = 0.5 * (uf'*uf);
    ceq(26*(M-1)+14) = 0; %Init.r*(yf' * f(xf,uf) + L);
    
end

