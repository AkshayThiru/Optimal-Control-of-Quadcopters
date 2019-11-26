function [q_s] = init_q (q_s)
    % initialize the states of the quadcopter
    
    % position of CoM
    q_s(1) = 0; % x
    q_s(2) = 0; % y
    q_s(3) = 0; % z
    
    % velocity of CoM
    q_s(4) = 0; % u
    q_s(5) = 0; % v
    q_s(6) = 0; % w
    
    % orientation: 
    q_s(7) = 1;
    q_s(8) = 0;
    q_s(9) = 0;
    q_s(10) = 0;
    
    % angular velocity in body fixed frame
    q_s(11) = 0; % p
    q_s(12) = 0; % q
    q_s(13) = 0; % r
end