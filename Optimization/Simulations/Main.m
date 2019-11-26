T = 100; % total time of simulation
sp = 1; % speed of plotting
step = 0.05;
nstep = step/sp;
t = 0; % current time
time = 0:nstep:T;

q_s = zeros(13,1);
q_s = init_q(q_s); % initialize state

pos = zeros(13,T/nstep+1);

vobj = VideoWriter('3D_Spiral','Motion JPEG AVI');
vobj.FrameRate = 200;  %just to be cinematic 
vobj.Quality=100; %no compression
open(vobj);

j = 1;
for i = 0 : nstep : T-nstep
   
   tic;
   % t = i
   
   [t_c,c_q_s] = ode45 (@(t_,y) Q.Control_Loop(t_,y), [t t+nstep/2 t+nstep], q_s);
   
   q_s = c_q_s(3,:)';
   pos(:,j+1) = q_s;
   
   Qplot(q_s);
   frame = getframe(1);
   writeVideo(vobj, frame);
   
   t_p = toc;
   if t_p >= step
       %'Solver too slow'
       %break;
   else
       % pause(step-t_p);
   end
   
   [Tr_r,Tr_v,Tr_a,Tr_y] = Trajectory(t);
   %figure(2)
   %hold on
   %plot(t, pos(3,j)', '.b')
   %plot(t, Tr_r(3), '.r');
   %hold off
   
   t = t+nstep;
   if mod(t,1) <= mod(t-nstep,1)
       (t-mod(t,1))
   end
   j=j+1;
end

close(vobj);

R_d = zeros(3,length(time)); idx = 1;
for t_t = time
    [r_d,~,~,~,~] = Trajectory(t_t);
    R_d(:,idx) = r_d;
    idx = idx +1;
end 
figure(1)
hold on
plot3(pos(1,:),pos(2,:),pos(3,:), '-b');
plot3(R_d(1,:),R_d(2,:),R_d(3,:), '--k');
title('Trajectory','Interpreter','latex');
legend('Actual Path','Desired Path');
grid on;
hold off

err = ((pos(1,:)-R_d(1,:)).^2+(pos(2,:)-R_d(2,:)).^2+(pos(3,:)-R_d(3,:)).^2).^(0.5);
figure(2)
hold on;
plot(time,err,'-b');
title('Trajectory','Interpreter','latex');
xlabel('Trajectory','Interpreter','latex');
ylabel('Trajectory','Interpreter','latex');
grid on;
hold off;