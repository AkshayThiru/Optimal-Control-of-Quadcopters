global xf x0;
xf = [0, 0]';
x0 = [7, -10]';
tf = 1;
l0 = [0, 0, tf]';

options = optimset('PlotFcns',@optimplotfval);
lm = fminsearch (@(l) test_model(l), l0, options);


opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[tp, sp] = ode45 (@(t,s) test_state(t,s), [0 lm(3)], [x0; lm(1:2)], opts);

figure(2)
hold on;
plot(sp(:,1), sp(:,2), '-b');
title('Optimal Trajectory','Interpreter','latex');
xlabel('x','Interpreter','latex');
ylabel('y','Interpreter','latex');
grid on;
axis equal;
hold off;