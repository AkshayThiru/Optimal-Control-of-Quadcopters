global Q Init M

F    = matfile('const.mat', 'Writable', true);
Q    = F.Q;
Init = F.Init;
M    = 2;

Init.xi(1:3) = [0; 0; 0]; % Initial position
Init.xd(1:3) = [1; 0; 1]; % Desired position

% Set bounds and initial guess
xL = -10 * ones(3, 1);      xU = 10 * ones(3, 1);   % Position bounds
sL = -20 * ones(3, 1);      sU = 20 * ones(3, 1);   % Velocity bounds
qL = -1 * ones(4, 1);       qU = ones(4, 1);        % Orientation bounds
wL = -100 * ones(3, 1);     wU = 100 * ones(3, 1);  % Angular velocity bounds
% yL = -Inf * ones(14, 1);  yU = Inf * ones(14, 1); % Lagrange multiplier bounds
tL = 2;                     tU = 10;                % Time bounds

% v0 = 0.5 * ones(26*(M-1)+14,1);
v0 = zeros(26*(M-1)+14,1);
vL = -10 * ones(26*(M-1)+14,1);
vU = 10 * ones(26*(M-1)+14,1);

if (M > 1)
    for i = 0:(M-2)
        v0(26*i+14:26*(i+1)) = [Init.xi(1:3) + (Init.xd(1:3)-Init.xi(1:3))*(i+1)/M; zeros(3,1); [1; 0; 0; 0]; zeros(3,1)];
        vL(26*i+14:26*(i+1)) = [xL; sL; qL; wL];
        vU(26*i+14:26*(i+1)) = [xU; sU; qU; wU];
    end
end

v0(end) = 3;
vL(end) = tL;
vU(end) = tU;

%%
% Solve using Pattern Search:
% Options for solver

% opt = optimoptions('patternsearch','Display','diagnose','PlotFcn',@psplotbestf,...
%             'Cache','on');
% 
% tic
% [zopt,~,~,out] = patternsearch(@(z) obj(z), v0, [], [], [], [], vL, vU, ...
%                @(z) constraints(z), opt) % x = patternsearch(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
% toc

% Solve NLP:
% Options for solver

% opt = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
%          'MaxFunEvals',Inf,'ConstraintTolerance',1e-3,'Hessian','off',...
%          'FiniteDifferenceStepSize',1e-1); % interior-point

opt = optimoptions('fmincon','Display','iter','Algorithm','interior-point',...
          'MaxFunEvals',Inf,'Hessian','off','MaxIter',31);

tic
[zopt,~,~,out,lmd] = fmincon(@(z) 0, V, [], [], [], [], vL, vU, ...
               @(z) constraints(z), opt) % x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
toc