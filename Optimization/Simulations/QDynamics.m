function [ d_q_s ] = QDynamics (~, q_s, u1, u2)
    % u1 - net force in upward direction, u2 - [3,1] net moment about center
    % along quadcopter axis, q_s - quad current state

    d_q_s = zeros (13,1);
    
    % rate of change of position:
    d_q_s(1:3) = q_s(4:6);
    
    q_s(7:10) = q_s(7:10)./sqrt(sum(q_s(7:10).^2));
    Rot = Q.QToR(q_s(7:10));
    
    % rate of change of velocity:
    d_q_s(4:6) = -[0 0 Q.g]' + Rot*[0 0 u1]'./Q.m;
    
    % rate of change of orientation:
    d_q_s(7:10) = 0.5*[-q_s(8) -q_s(9) -q_s(10); q_s(7) -q_s(10) q_s(9); q_s(10) q_s(7) -q_s(8); -q_s(9) q_s(8) q_s(7)]*Rot*q_s(11:13);
    
    % rate of change of angular velocity:
    d_q_s(11:13) = Q.I^-1 * ( u2 - cross(q_s(11:13), Q.I*q_s(11:13)) );
    
end