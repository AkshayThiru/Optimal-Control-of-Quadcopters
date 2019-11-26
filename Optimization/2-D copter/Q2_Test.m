global xf x0;
xf = zeros(6,1);
x0 = [0.2 0 0 0 0 0]';
tf = 0.7;
l0 = [-2.8049 -0.4353 -1.1259 -0.3702 0.0141 -0.0396 tf]';
% l0 = [zeros(1, 6) tf]';

options = optimset('PlotFcns',@optimplotfval,'MaxFunEvals',4000,'MaxIter',4000,'TolFun',10^-3);
lm = fminsearch (@(l) Q2.model(l), l0, options);

% rng default;
% ms = MultiStart('TolFun',1e-4,'UseParallel',true);
% gs = GlobalSearch(ms,'Display','iter');
% gs.NumTrialPoints = 5000;
% opts = optimoptions(@fmincon,'Display','iter');
% problem = createOptimProblem('fmincon','x0',l0,'objective',@(l) Q2.model(l),'lb',[-5*ones(6,1); 0.1],'ub',[5*ones(6,1); 5],'options',opts);
% lm = run(gs,problem);

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[tp, sp] = ode45 (@(t,s) Q2.state(t,s), [0 lm(7)], [x0; lm(1:6)], opts);
% [tp, sp] = ode45 (@(t,s) Q2.state(t,s), [0 1], [x0; [0 0 -1 -0.5 0 0]'], opts);

figure(2)
hold on;
plot(sp(:,1), sp(:,3), '-b');
title('Local Optimal Trajectory','Interpreter','latex');
xlabel('x','Interpreter','latex');
ylabel('y','Interpreter','latex');
grid on;
axis equal;
hold off;