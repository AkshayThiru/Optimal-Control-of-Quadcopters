function [] = Qplot (q_s)
    % h is the 3D plot handle, and q_state is the [13,1] state
    % variable
    
    Rot = Q.QToR(q_s(7:10));
    arm_x = Rot*[1 0 0]'*Q.L;
    arm_y = Rot*[0 1 0]'*Q.L;
    arm_z = Rot*[0 0 1]'*Q.H;
    
    figure(1) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clf;
    hold on
    % Center
    plot3 ( q_s(1), q_s(2), q_s(3), '.r');
    % x-Arms
    plot3 ([q_s(1) q_s(1)+arm_x(1)], [q_s(2) q_s(2)+arm_x(2)], [q_s(3) q_s(3)+arm_x(3)], '-ko', 'MarkerSize', 2);
    plot3 ([q_s(1) q_s(1)-arm_x(1)], [q_s(2) q_s(2)-arm_x(2)], [q_s(3) q_s(3)-arm_x(3)], '-ko', 'MarkerSize', 2);
    
    % y-Arms
    plot3 ([q_s(1) q_s(1)+arm_y(1)], [q_s(2) q_s(2)+arm_y(2)], [q_s(3) q_s(3)+arm_y(3)], '-ko', 'MarkerSize', 2);
    plot3 ([q_s(1) q_s(1)-arm_y(1)], [q_s(2) q_s(2)-arm_y(2)], [q_s(3) q_s(3)-arm_y(3)], '-ko', 'MarkerSize', 2);
    
    % z-Arms
    plot3 ([q_s(1) q_s(1)+arm_z(1)], [q_s(2) q_s(2)+arm_z(2)], [q_s(3) q_s(3)+arm_z(3)], '-ko', 'MarkerSize', 2);
    
    xlim([-3 3]);
    ylim([-3 3]);
    zlim([0 6]);
    grid on;
    % view(0,0);
    view(45,30);
    drawnow limitrate
    hold off
    %axis equal;
    
end