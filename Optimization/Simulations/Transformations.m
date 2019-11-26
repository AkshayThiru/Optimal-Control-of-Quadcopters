syms r p y; % Angles
syms R P Y; % Transformations
syms r1 r2 r3; % Rotation matrices corresponding to e1, e2, and e3 respectively

Y = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
P = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
R = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];

T12 = [0 0 1; 1 0 0; 0 1 0];
T23 = T12; T31 = T12;
T21 = [0 1 0; 0 0 1; 1 0 0];
T32 = T21; T13 = T21;

r1 = Y*P*R;
r2 = r1*T12;
r3 = r1*T13;