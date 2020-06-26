function [val] = obj (v) % v: variables (26*(M-1)+14)

    global M Init

    % Initial conditions for all times
    z0 = [Init.xi; v(1:end-1)]; % size: 26*M
    
    % Final states
    [~,z] = ode45(@(t,z) dyn(z), [0, v(end)/M], z0);
    zf = z(end,1:end)'; % size: 26*M: for t = tf/M
    
    val = v(end)^2 + Init.r*sum((zf(26*(M-1)+1:26*(M-1)+13)-Init.xd).^2);
end

