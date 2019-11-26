classdef Q
    
    properties (Constant)
        m = 1;
        L = 0.25; % Arm Length
        H = 0.1; % Height
        I = [0.0125 0 0; 0 0.0125 0; 0 0 0.025];
        g = 9.81;
        k_M = 0.09525;
    end
   
    methods (Static)
        
        function [ Rot ] = QToR( q ) % q - [4,1] quaternion, Rot - [3,3] rotation matrix
            
            q = q./sqrt(sum(q.^2));
            
            q_c = zeros (3,3);
            q_c(3,2) = q(2);
            q_c(1,3) = q(3);
            q_c(2,1) = q(4);
            q_c = q_c - q_c';

            Rot = eye(3) + 2*q_c*q_c + 2*q(1)*q_c;

        end
        
        function [ E1 ] = QToE1 ( q ) % q - [4,1] quaternion, E1 - [3,1] euler angles set-1
            
            E1 = zeros(3,1);
            E1(1) = atan2( 2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2)); % Roll
            E1(2) = asin( 2*(q(1)*q(3)-q(2)*q(4)) ); % Pitch
            E1(3) = atan2( 2*(q(1)*q(4)+q(2)*q(3)), 1-2*(q(3)^2+q(4)^2)); % Yaw
            
        end
        
        function [ q ] = E1ToQ ( E1 ) % E1 - [3,1] euler angles set-1, q - [4,1] quaternion
            
            q(1) = cos(E1(1)/2)*cos(E1(2)/2)*cos(E1(3)/2)+sin(E1(1)/2)*sin(E1(2)/2)*sin(E1(3)/2);
            q(2) = sin(E1(1)/2)*cos(E1(2)/2)*cos(E1(3)/2)-cos(E1(1)/2)*sin(E1(2)/2)*sin(E1(3)/2);
            q(3) = cos(E1(1)/2)*sin(E1(2)/2)*cos(E1(3)/2)+sin(E1(1)/2)*cos(E1(2)/2)*sin(E1(3)/2);
            q(4) = cos(E1(1)/2)*cos(E1(2)/2)*sin(E1(3)/2)-sin(E1(1)/2)*sin(E1(2)/2)*cos(E1(3)/2);
            
        end
        
        function [ E1 ] = RToE1( R ) % R - [3,3] rotation matrix, E1 - [3,1] euler angles set-1: roll, pitch, yaw
            
            E1(2) = asin(-R(3,1)); % Pitch
            
            if R(3,1) == 1 || R(3,1) == -1
                E1(1) = atan2(R(1,2), R(1,3)); % Roll
                E1(3) = 0; % Yaw
            else
                E1(1) = atan2(R(3,2), R(3,3)); % Roll
                E1(3) = atan2(R(2,1), R(1,1)); % Yaw
            end
            
        end
        
        function [ R ] = E1ToR ( E1 ) % E1 - [3,1] euler angles set-1: roll, pitch, yaw, R - [3,3] rotation matrix
            
            R_ = [1 0 0; 0 cos(E1(1)) -sin(E1(1)); 0 sin(E1(1)) cos(E1(1))]; % Lab Roll
            P_ = [cos(E1(2)) 0 sin(E1(2)); 0 1 0; -sin(E1(2)) 0 cos(E1(2))]; % Lab Pitch
            Y_ = [cos(E1(3)) -sin(E1(3)) 0; sin(E1(3)) cos(E1(3)) 0; 0 0 1]; % Lab Yaw
            R = Y_*P_*R_;
            
        end
        
        function [ E2 ] = E1ToE2 ( E1 ) % E1 - [3,1] euler angles set-1, E2 - [3,1] euler angles set-2
            
            E2(2) = asin( -cos(E1(2))*sin(E1(1)) ); % Pitch
            E2(1) = atan2( cos(E1(2))*cos(E1(1)), -sin(E1(2)) ); % Roll
            E2(3) = atan2( cos(E1(1))*cos(E1(3))+sin(E1(2))*sin(E1(1))*sin(E1(3)), cos(E1(3))*sin(E1(2))*sin(E1(1))-cos(E1(1))*sin(E1(3)) ); % Yaw
            
        end
        
        function [ E3 ] = E1ToE3 ( E1 ) % E1 - [3,1] euler angles set-1, E3 - [3,1] euler angles set-3
            
            E3(2) = asin( -cos(E1(2))*cos(E1(1)) ); % Pitch
            E3(1) = atan2( -sin(E1(2)), cos(E1(2))*sin(E1(1)) ); % Roll
            E3(3) = atan2( cos(E1(1))*sin(E1(2))*sin(E1(3))-cos(E1(3))*sin(E1(1)), sin(E1(1))*sin(E1(3))+cos(E1(1))*cos(E1(3))*sin(E1(2)) ); % Yaw
            
        end
        
        function [ E1 ] = E2ToE1 ( E2 )
            E1 = E1ToE3(E2);
        end
        
        function [ E1 ] = E3ToE1 ( E3 )
            E1 = E1ToE2(E3);
        end
        
        function [ E3 ] = E2ToE3 ( E2 )
            E3 = E1ToE2(E2);
        end
        
        function [ E2 ] = E3ToE2 ( E3 )
            E2 = E1ToE3(E3);
        end
        
        function [u1, u2] = ForceToI (F) % F - Force [4,1] Force, u1 - Net force, u2 - [3,1] Net moment
            
            u1 = sum(F);
            u2 = [(F(2)-F(4))*L (F(3)-F(1))*L (F(2)+F(4)-F(1)-F(3))*k_M]';
           
        end
        
        function [F] = IToForce (u) % u is the desired [4,1] force moment vector, F is the Force required
            
            F(1) = 1/4 * (u(1) - 2*u(3)/L - u(4)/k_M);
            F(2) = 1/4 * (u(1) + 2*u(2)/L + u(4)/k_M);
            F(3) = 1/4 * (u(1) + 2*u(3)/L - u(4)/k_M);
            F(4) = 1/4 * (u(1) - 2*u(2)/L + u(4)/k_M);
            
        end
        
        function [d_q_s] = Control_Loop (t, q_s)
            % gets Trajectory, control inputs, and returns the rate of
            % change of states of the quad
            
            q_s(7:10) = q_s(7:10)./sqrt(sum(q_s(7:10).^2));
            
            [r_d, v_d, a_d, y_d, y_r] = Trajectory(t);
            [u1, u2] = Controller.Control(t, q_s, r_d, v_d, a_d, y_d, y_r);
            d_q_s = QDynamics(t, q_s, u1, u2);
            
        end
        
    end
        
end