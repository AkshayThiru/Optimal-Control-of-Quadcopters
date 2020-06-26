Q = struct;
Q.m = 1; % Mass
Q.l = 0.2; % Arm length (0.5 * Rotor-to-rotor distance)
Q.g = [0; 0; 9.81]; % Gravity
Q.k = Q.m * Q.g(3); % Maximum thrust for one motor
Q.M = Q.l/sqrt(2); % Thrust to Moment factor
Q.I = [Q.m*(Q.l)^2/24, 0             , 0             ;
       0             , Q.m*(Q.l)^2/24, 0             ;
       0             , 0             , Q.m*(Q.l)^2/12]; % Moment of Inertia

Init = struct;
Init.r  = 1000; % Constrint enforcement factor
Init.xi = [0; 0; 0; % Initial position
           0; 0; 0; % Initial velocity
           1; 0; 0; 0; % Initial orientation
           0; 0; 0]; % Initial angular velocity
Init.xd = [0; 0; 1; % Desired position
           0; 0; 0; % Desired velocity
           1; 0; 0; 0; % Desired orientation
           0; 0; 0]; % Desired angular velocity

save const.mat Q Init -v7.3;
clear Q Init