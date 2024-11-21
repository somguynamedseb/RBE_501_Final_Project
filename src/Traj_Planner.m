classdef Traj_Planner < handle
    % Trajectory Planner Class
    %  Contains methods for generating trajectories for the robot arm
    %  Uses cubic and quintic trajectories
    % Team 19 A2023 WPI RBE 3001
    properties
    end
    
    methods
        % Solves a cubic trajectory between two point
        % Takes in start and end times (t0, tf), start and end positions
        % (q0, qf), and start and end velocities (v0, vf)
        % Returns a 4x1 array of coefficients a0, a1, a2, a3
        function a = cubic_traj(self, t0, tf,q0, qf, v0, vf)
            M = [1 t0 t0^2 t0^3;
                0 1 2*t0 3*t0^2;
                1 tf tf^2 tf^3;
                0 1 2*tf 3*tf^2];
            
            B = [q0 v0 qf vf].';
            
            a = M\B;
        end
        
        % Solves a quintic trajectory between two points
        % Takes in start and end times (t0, tf), start and end positions
        % (q0, qf), start and end velocities (v0, vf), and start and end
        % accelerations (a0 and af)
        % Returns a 6x1 array of coefficients a0, a1, a2, a3, a4, a5
        function a = quintic_traj(self, t0, tf, q0, qf, v0, vf, a0, af)
            M = [1 t0 t0^2 t0^3 t0^4 t0^5;
                0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
                0 0 2 6*t0 12*t0^2 20*t0^3;
                1 tf tf^2 tf^3 tf^4 tf^5;
                0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
                0 0 2 6*tf 12*tf^2 20*tf^3];
            
            B = [q0 v0 a0 qf vf af].';
            
            a = M\B;
        end
        
        
        % Helper function for generating cubic trajectories of 4 variables
        % Input is the final interpolation time (start is assumed at zero), initial and desired positions (both 1x4 arrays)
        % Outputs a 4x4 matrix of trajectory coefficients for each joint and
        % each row represents the coefficients for a joint
        % or can be used to...
        % Outputs a 4x4 matrix with coefficients for X, Y, Z, and Alpha
        function M = prepareCubicTraj(self, time_final, pos_initial, pos_final)
            M = zeros(4,4);
            for i = 1:4
                M(i, :) = self.cubic_traj(0, time_final, pos_initial(i), pos_final(i), 0, 0).';
            end
        end
        
        % Helper function for generating quintic trajectories of 4 variables
        % Input is the final interpolation time (start is assumed at zero), initial and final positions (both 1x4 arrays)
        % Outputs a 4x6 matrix with coefficients for the 4 interpolated variables
        % which are either the 4 joins, or the X, Y, Z, and Alpha values
        function M = prepareQuinticTraj(self, time_final, pos_initial, pos_final)
            M = zeros(4,6);
            for i = 1:4
                M(i, :) = self.quintic_traj(0, time_final, pos_initial(i), pos_final(i), 0, 0, 0, 0).';
            end
        end

        


        
    end
end