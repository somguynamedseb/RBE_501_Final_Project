% MATLAB
% (c) 2023 Jakub Jandus, Team 19
classdef Model
    %the robot arm plotting class
    properties
        figure_ref; % obj reference to the figure
        frames_ref; %object reference to the frames
        ee_text_ref;
        ee_vel_ref; %velocity quiver reference object
        
        robot_ref;

        %link lengths
        L0;
        L1;
        L2;
        L3;
        L4;
    end

    methods
        function self = Model(robot_reference)
        % move the DH and link lengths here
        % have it reference the ROBOT class for the dh2fk
        % and the L1, ... and DH params
            self.robot_ref = robot_reference;

            % Get link lengths in mm
            self.L0 = 36.076;
            self.L1 = 96.326 - self.L0;
            self.L2 = sqrt(128^2 + 24^2);
            self.L3 = 124;
            self.L4 = 133.4; 
                        
            % create the plot starting in home position            
            q = [0; 0; 0; 0];
            [self.figure_ref, self.frames_ref, self.ee_text_ref, self.ee_vel_ref] = self.plot_arm(q);
            
            %self.update_plot(q, self.figure_ref, self.frames_ref);
        end

        %% -----------------------------

        function T = dh2mat(self, q) %SELF!!!
                T = [cosd(q(1)) -sind(q(1))*cosd(q(4)) sind(q(1))*sind(q(4)) q(3)*cosd(q(1));
                     sind(q(1)) cosd(q(1))*cosd(q(4)) -cosd(q(1))*sind(q(4)) q(3)*sind(q(1));
                     0 sind(q(4)) cosd(q(4)) q(2);
                     0 0 0 1];
        end


        % Takes a 1x4 vector q representing the joing angle values
        % Graphs the stick figure of the robot arm
        % Returns a reference to the plot window
        function [fig, FrameQuivers, text_ref, VelQuiver] = plot_arm(self, q)     
            % Get the DH parameter table for the arm
            DH = [q(1) self.L1 0 -90;
                  q(2)-acosd(24 / self.L2) 0 self.L2 0;
                  q(3)+acosd(24 / self.L2) 0 self.L3 0;
                  q(4) 0 self.L4 0];                 
        
            % initialize variable to store all the T0n transformations
            % 4x4 matrix, 6 times
            FramesRaw = zeros(4, 4, 6);
            FrameQuivers = zeros(3, 6);
        
            %-----------
            % Get T00
            FramesRaw(:, :, 1) = eye(4);
        
            % Get T01
            T01 = self.dh2mat([0 self.L0 0 0]);   %SELF!!!   
            FramesRaw(:, :, 2) = T01;
        
            % Get T02
            T02 = T01 * self.dh2mat(DH(1,:));   %SELF!!!   
            FramesRaw(:, :, 3) = T02;
        
            % Get T03
            T03 = T02 * self.dh2mat(DH(2,:));   %SELF!!!   
            FramesRaw(:, :, 4) = T03;
        
            % Get T04
            T04 = T03 * self.dh2mat(DH(3,:));   %SELF!!!   
            FramesRaw(:, :, 5) = T04;
        
            % Get T05
            T05 = T04 * self.dh2mat(DH(4,:));   %SELF!!!   
            FramesRaw(:, :, 6) = T05;
        
            %----------- PLOT
            %FramesRaw
        
            X2Graph = [0];
            Y2Graph = [0];
            Z2Graph = [0];
            
            for index = 1:6
                X2Graph(index) = FramesRaw(1, 4, index);
                Y2Graph(index) = FramesRaw(2, 4, index);
                Z2Graph(index) = FramesRaw(3, 4, index);      
            end
            
            %plot the 4-dof arm
            fig = plot3(X2Graph(:), Y2Graph(:), Z2Graph(:), '-o', 'LineWidth', 3, ...
                'MarkerSize', 6, 'MarkerFaceColor', 'b');
            view([(-37.5 + 160) 25])
            
            %plot the end effector XYZ position
            text_ref = text(X2Graph(end), Y2Graph(end), Z2Graph(end), [' (', ...
                (num2str(round(X2Graph(end), 4), 4)), ', ', ...
                (num2str(round(Y2Graph(end), 4), 4)), ', ', ...
                (num2str(round(Z2Graph(end), 4), 4)), ') ']);
            
            %plot the Frame XYZ at each joint
            hold on
            for index = 1:6
                % color the TCP a different color than the rest
                FrameQuivers(:, index) = self.FrameQuiver3D(FramesRaw(:, :, index), 60);
            end
            
            % construct the velocity quiver, zero size for now
            VelQuiver = quiver3(X2Graph(end), Y2Graph(end), Z2Graph(end),0,0,0, ...
                Color='m', AutoScale='on', AutoScaleFactor=2.5, LineWidth=2);

            axis equal
            grid on          
            hold off
            
            %graph settings
            xlabel("X Axis (mm)");
            ylabel("Y Axis (mm)");
            zlabel("Z Axis (mm)");
            title("HALL 4000");
            axis([-400 400 -400 400 -50 450]);       
        end
        
        % Update the live plot with the current joint positions and
        % Jacobian velocities
        function update_plot(self, q, J)
            % Get the DH parameter table for the arm
            DH = [q(1) self.L1 0 -90;
                  q(2)-acosd(24 / self.L2) 0 self.L2 0;
                  q(3)+acosd(24 / self.L2) 0 self.L3 0;
                  q(4) 0 self.L4 0];
          
        
            % initialize variable to store all the T0n transformations
            % 4x4 matrix, 6 times
            FramesRaw = zeros(4, 4, 6);
        
            %-----------
            % Get T00
            FramesRaw(:, :, 1) = eye(4);
        
            % Get T01
            T01 = self.dh2mat([0 self.L0 0 0]);   %SELF!!!   
            FramesRaw(:, :, 2) = T01;
        
            % Get T02
            T02 = T01 * self.dh2mat(DH(1,:));   %SELF!!!   
            FramesRaw(:, :, 3) = T02;
        
            % Get T03
            T03 = T02 * self.dh2mat(DH(2,:));   %SELF!!!   
            FramesRaw(:, :, 4) = T03;
        
            % Get T04
            T04 = T03 * self.dh2mat(DH(3,:));   %SELF!!!   
            FramesRaw(:, :, 5) = T04;
        
            % Get T05
            T05 = T04 * self.dh2mat(DH(4,:));   %SELF!!!   
            FramesRaw(:, :, 6) = T05;
        
            %----------- PLOT
            %FramesRaw
        
            X2Graph = [0];
            Y2Graph = [0];
            Z2Graph = [0];
            
            for index = 1:6
                X2Graph(index) = FramesRaw(1, 4, index);
                Y2Graph(index) = FramesRaw(2, 4, index);
                Z2Graph(index) = FramesRaw(3, 4, index);      
            end
            
            % update the data for the 4-dof arm
            set(self.figure_ref, 'XData', X2Graph, 'YData', Y2Graph, 'ZData', Z2Graph);
            
            set(self.ee_text_ref,'Position', [X2Graph(end) Y2Graph(end) Z2Graph(end)]);
            set(self.ee_text_ref, 'String',[' (', ...
                (num2str(round(X2Graph(end), 4), 4)), ', ', ...
                (num2str(round(Y2Graph(end), 4), 4)), ', ', ...
                (num2str(round(Z2Graph(end), 4), 4)), ') ']);                     
            
            % Update velocity vector (maybe)
            % check that J has actually been passed in 
            if length(J) == 6
                % update the velocity vector if there is anything to update
                vel_lin_X = J(1,1);
                vel_lin_Y = J(2,1);
                vel_lin_Z = J(3,1);

                set(self.ee_vel_ref, 'XData', X2Graph(end), 'YData', Y2Graph(end), 'ZData', ...
                    Z2Graph(end), 'UData', vel_lin_X, 'VData', vel_lin_Y, 'WData', vel_lin_Z);
            end

            for joint_index = 1:6                
                % set the quiver data by parsing in the current transformation matrix    
                x_coord = FramesRaw(1, 4, joint_index);
                y_coord = FramesRaw(2, 4, joint_index);
                z_coord = FramesRaw(3, 4, joint_index);
        
                for quiver_index = 1:3     
                    %unit vector of X1-axis, then Y1, then Z1 axis
                    U_coord = FramesRaw(1, quiver_index, joint_index);
                    V_coord = FramesRaw(2, quiver_index, joint_index); 
                    W_coord = FramesRaw(3, quiver_index, joint_index);
         
                    %quiver3(x_coord, y_coord, z_coord, U_coord, V_coord, W_coord, scale, color);                         
                    set(self.frames_ref(quiver_index, joint_index), 'XData', x_coord, 'YData', ...
                        y_coord, 'ZData', z_coord, 'UData', U_coord, ...
                        'VData', V_coord, 'WData', W_coord); 
                end
            end
        end

        % takes in a 4x4 Transformation Matrix
        % Outputs a 3x6 matrix representing the 3 Quiver3D of the frame X, Y, and Z
        %   also directly plots the quivers
        function Quiv = FrameQuiver3D(self, TransMatrix, scale)        
            Quiv = zeros(3, 1);
            x_coord = TransMatrix(1, 4);
            y_coord = TransMatrix(2, 4);
            z_coord = TransMatrix(3, 4);
        
            for idx = 1:3
                %unit vector of X1-axis, then Y1, then Z1 axis
                U_coord = TransMatrix(1, idx);
                V_coord = TransMatrix(2, idx);  
                W_coord = TransMatrix(3, idx);

                if idx == 3
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, V_coord, W_coord, scale, Color='b');
                elseif idx == 2            
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, ...
                        V_coord, W_coord, scale, Color = 'g', LineStyle="-.");    %green
                else                    % [rnd_colorR, rnd_colorG, rnd_colorB],
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, ...
                        V_coord, W_coord, scale,  Color = 'r', LineStyle="-.");    %red
                end              
            end
        end
        
    end % end methods
end % end class 