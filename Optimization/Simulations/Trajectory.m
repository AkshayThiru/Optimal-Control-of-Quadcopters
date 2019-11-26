function [r_d, v_d, a_d, y_d, y_r] = Trajectory (t)
    % r_d - [3,1] desired position, v_d - [3,1] desired velocity, a_d -
    % [3,1] desired acceleration, y_d - desired yaw; at time t
    
    % Hover
%     r_d = [0 0 1+sin(t)]';
%     v_d = [0 0 cos(t)]';
%     a_d = [0 0 -sin(t)]';
%     y_d = 0;
%     y_r = 0;

    % Waypoint - Run Time >= 10 sec: 3-D
%     r_d = 2*[((mod(t,50)>=6.25&&mod(t,50)<=18.75)|(mod(t,50)>=31.25&&mod(t,50)<=43.75)) (mod(t,50)>=12.5&&mod(t,50)<=37.5) (mod(t,50)<=25)]';
%     v_d = [0 0 0]';
%     a_d = [0 0 0]';
%     y_d = 0;
%     y_r = 0;

    % Rectangle - Run Time >= 10 sec: 2-D
%     r_d = [(mod(t,20)<=10)*2 0 (mod(t+5,20)>=10)*2]';
%     v_d = [0 0 0]';
%     a_d = [0 0 0]';
%     y_d = 0;
%     y_r = 0;
    
    % Circle - Run Time >= 20 sec: 2-D
%     k = 2*pi/10;
%     r_d = [cos(k*t) 0 1+sin(k*t)]';
%     v_d = [-k*sin(k*t) 0 k*cos(k*t)]';
%     a_d = [-k^2*cos(k*t) 0 -k^2*sin(k*t)]';
%     y_d = 0;
%     y_r = 0;

    % Wave - Run Time >= 20 sec: 2-D
%     k = 2*pi/10;
%     r_d = [cos(k*t) 1 t/10-5]';
%     v_d = [-k*sin(k*t) 0 1/10]';
%     a_d = [-k^2*cos(k*t) 0 0]';
%     y_d = 0;
%     y_r = 0;
    
    % Spiral - Run Time >= 100 sec: 3-D
    k = 2*pi/20;
    r_d = [2*cos(t*k) 2*sin(t*k) t/20+0.1*sin(4*t*k)]';
    v_d = [-2*k*sin(t*k) 2*k*cos(t*k) 1/20+(0.4*k)*cos(4*t*k)]';
    a_d = [-2*k^2*cos(t) -2*k^2*sin(t) -(0.4*k)^2*sin(4*t*k)]';
    y_d = 0;
    y_r = 0;
    
end