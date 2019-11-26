classdef Controller
    
    properties (Constant)
        % contains all the parameters required for controller
        
        % Linear Controller Parameters
        K_p_r = diag([0.25 0.25 0.25], 0);
        K_d_r = diag([1 1 1], 0);
        K_p_phi = 100; K_p_the = 100; K_p_psi = 100;
        K_d_phi = 20; K_d_the = 20; K_d_psi = 20;
        
    end
    
    methods (Static)
        
        function [u1, u2] = Control (t, q_s, r_d, v_d, a_d, y_d, y_r)
            % u1 - Net force, u2 - [3,1] Net moment, q_s - [13,1] quad state
            
            u2 = zeros(3,1);
            % insert control code
            
            % Linear Controller - 3D
            a_ac = a_d + Controller.K_d_r*(v_d-q_s(4:6)) + Controller.K_p_r*(r_d-q_s(1:3));
            u1 = Q.m*Q.g + Q.m*(a_ac(3));
            
            e1 = Q.QToE1(q_s(7:10));

            theta_d = 1/(Q.g) * (a_ac(1)*cos(e1(3))+a_ac(2)*sin(e1(3)));
            phi_d = 1/(Q.g) * (a_ac(1)*sin(e1(3))-a_ac(2)*cos(e1(3)));
            
            u2(1) = Controller.K_p_phi*(phi_d-e1(1))+Controller.K_d_phi*(-q_s(11));
            u2(2) = Controller.K_p_the*(theta_d-e1(2))+Controller.K_d_the*(-q_s(12));
            u2(3) = Controller.K_p_psi*(y_d-e1(3))+Controller.K_d_psi*(y_r-q_s(13));
            u2 = Q.I*u2;
            
            % Non-Linear Controller - 3D

            % end
            
            % check maximum forces and anti-windup for I terms
        end
        
    end
    
end