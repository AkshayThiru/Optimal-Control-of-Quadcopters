function [ds] = test_state (~, s) % s(1:2) = x, s(3:4) = l
    ds = zeros(4,1);
%     u = -sign(s(4)); % minimum time
%     u = -0.5*(s(4)); % minimum energy, unbounded input
    if abs(0.5*(s(4))) <= 0.2
        u = -0.5*(s(4));
    else
        u = -0.2*sign(s(4));
    end
    
    ds(1) = s(2);
    ds(2) = -s(1) + u;
    ds(3) = -s(4);
    ds(4) = s(3);
end