% MATLAB
% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        goalJS; % Stores end goal joint positions [1x4 double]
        
        % Link lengths
        L0;
        L1;
        L2;
        L3;
        L4;
    end
    
    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);
            
            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
            
            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            
            % Set end goal joint positions
            self.goalJS = [0, 0, 0, 0];
            
            % Set up link lengths in mm
            self.L0 = 36.076;
            self.L1 = 96.326 - self.L0;
            self.L2 = sqrt(128^2 + 24^2);
            self.L3 = 124;
            self.L4 = 133.4;
        end
        
        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end
        
        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end
            
            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;
            
            %             disp("time");
            %             disp(time_ms);
            %             disp("acc time");
            %             disp(acc_time_ms);
            
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end
        
        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end
        
        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end
        
        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'}
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end
            
            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);
            
            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end
        
        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end
        
        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
        
        % Sends joint values directly to the servos without interpolation
        % values [1x4 double] - The values for each joint
        function servo_jp(self, values)
            % update goal joint positions
            self.goalJS = values;
            
            % change writet time to 0 to remove interpolation
            self.writeTime(0.05);
            
            % write joint values
            self.writeJoints(values);
        end
        
        % Sends joint values directly to the servos with given
        % interpolation time
        % values [1x4 double] - The values for each joint
        % intrpl_time_ms [1x1 double] - The interpolation time in millisseconds
        function interpolate_jp(self, values, interTime)
            % update goal joint positions
            self.goalJS = values;
            
            % set the movement time
            self.writeTime(interTime / 1000);
            
            % write the values
            self.writeJoints(values);
        end
        
        % Gets the joint positions and velocities and returns what the user
        % wants of those data. Data that is not requested is returned as 0s
        % GETPOS [boolean] - True if user wants position data, false
        % otherwise
        % GETVEL [boolean] - True if user wants velocity data, false
        % otherwise
        function fast_readings = measured_js(self, GETPOS, GETVEL)
            % Make array to store data
            fast_readings = zeros(2,4);
            
            % copy only the first if position data requested
            if GETPOS
                fast_readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end
            
            % copy only the second if velocity data requested
            if GETVEL
                fast_readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
                
                % Subtract static velocity offsets found experimentally
                offsets = [0, 61.8300, 65.9520, 0];
                fast_readings(2, :) = fast_readings(2, :) - offsets;
            end
        end
        
        % Get current joint setpoint positions and return them
        function set_position = setpoint_js(self)
            % return the current setpoint joint positions
            % for faster runtime directly call the bulkReadWrite function as in getJointsReadings(self)
            set_position = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end
        
        % Retrieves the class variable goalJS, which is the robot's current
        % goal joint values
        function goal_position = goal_js(self)
            goal_position = self.goalJS;
        end
        
        % Takes data from measured_js() and returns a 4x4 homogeneous transformation
        % matrix based upon the current joint positions in degrees
        function tranformation_Matrix = measured_cp(self)
            joint_Vals = measured_js(self,true,false);
            joint_Vals = joint_Vals(1,:).';
            
            tranformation_Matrix = fk3001(self,joint_Vals);
        end
        
        % Takes data from setpoint_js() and returns a 4x4 homogeneous transformation matrix
        % based upon the current joint set point positions in degrees.
        % If interpolation is being used and you request this during motion it will return the current intermediate set point
        function tranformation_Matrix = setpoint_cp(self)
            joint_Vals = setpoint_js(self).';
            tranformation_Matrix = fk3001(self,joint_Vals);
        end
        
        % Takes data from goal_js() and returns a 4x4 homogeneous transformation matrix
        % based upon the commanded end of motion joint set point positions in degrees
        function tranformation_Matrix = goal_cp(self)
            joint_Vals = goal_js(self).';
            tranformation_Matrix = fk3001(self,joint_Vals);
        end
        
        % Takes in a 1x4 array corresponding to a row of the DH parameter table and
        % generates and returns the corresponding symbolic 4x4 homogeneous transformation matrix
        function T = dh2mat(self, q)
            T = [cosd(q(1)) -sind(q(1))*cosd(q(4)) sind(q(1))*sind(q(4)) q(3)*cosd(q(1));
                sind(q(1)) cosd(q(1))*cosd(q(4)) -cosd(q(1))*sind(q(4)) q(3)*sind(q(1));
                0 sind(q(4)) cosd(q(4)) q(2);
                0 0 0 1];
        end
        
        % Takes in an nx4 matrix representing a DH table and the
        % transformation matrix from frame 0 to frame 1
        % Returns the composite symbolic transformation matrix from frame 0
        % to the end effector frame  
        function T = dh2fk(self, q, t01)
            theta = sym('theta%d', [1 size(q, 1)]);
            d = sym('d%d', [1 size(q, 1)]);
            a = sym('a%d', [1 size(q, 1)]);
            alph = sym('alph%d', [1 size(q, 1)]);
            
            T = t01;
            
            % Iterate through rows of DH and multiply T by the next
            % transformation
            for i = 1:size(q, 1)
                T = T * self.dh2mat([theta(i) d(i) a(i) alph(i)]);
            end
        end
        
        % Takes an nx1 vector representing the n joint angles of the arm (4)
        % Returns a 4x4 transformation matrix representing the position and
        % orientation of the tip frame with respect to the base frame
        function T = fk3001(self, q)
            % Get the DH parameter table for the arm
            DH = [q(1) self.L1 0 -90;
                q(2)-acosd(24 / self.L2) 0 self.L2 0;
                q(3)+acosd(24 / self.L2) 0 self.L3 0;
                q(4) 0 self.L4 0];
            
            % Get T00
            T = eye(4);
            
            % Get T01
            T = T * self.dh2mat([0 self.L0 0 0]);
            
            % Get T02, T03, T04, T05
            for index = 1:4
                T = T * self.dh2mat(DH(index,:));
            end
        end
        
        % Takes in a 1x4 vector [x, y, z, alpha] coordinates of the end effector
        % Outputs a 1x4 vector of joint angles to reach the desired position
        % theta_1 limits -90, 90
        % theta_2 limits -90, 90
        % theta_3 limits -100, 90
        % theta_4 limits -100, 120
        function q = ik3001(self,EE)
            
            % extracted end effector into separate variables
            Xe = EE(1);
            Ye = EE(2);
            Ze = EE(3);
            alpha_EE0 = EE(4);
            
            
            % Coordinates to joint 2 are known, this is the Z height of joint 2
            Z2 = self.L0 + self.L1;
            
            % extensive use of try catch blocks to allow for the program to continue if only one of the configurations is unreachable
            
            try
                % Theta 1 is angle from x axis to (x, y) of EE only in the XY plane
                re = sqrt(Xe^2 +Ye^2);
                theta_1_pos = atan2d(sqrt(1-(Xe/re)^2) , (Xe/re));
                theta_1_neg = atan2d(-sqrt(1-(Xe/re)^2) , (Xe/re));
                
                % theta_1 depends on the desired Y position of the end effector
                % if the end effector is in the +Y direction, then theta_1 is positive
                % if the end effector is in the -Y direction, then theta_1 is negative
                if(Ye > 0)
                    theta_1 = theta_1_pos;
                else
                    theta_1 = theta_1_neg;
                end
                
                if(theta_1 > 90 || theta_1 < -90)
                    error("Theta_1 out of joint allowed range")
                end
                
            catch ER
                % an error occured, read and return the current joint values
                warning("Theta_1 " + ER.message);
            end
            
            % configuration elbow positive
            try
                % Theta 2 calculations:
                Z4 = self.L4 * sind(alpha_EE0) - Z2 + Ze;
                r4 = re - self.L4 * cosd(alpha_EE0);
                c = sqrt(r4^2 +Z4^2);
                
                beta_pos = atan2d(sqrt(1-((self.L2^2 + c^2 - self.L3^2)/(2*self.L2*c))^2) ,((self.L2^2 + c^2 - self.L3^2)/(2*self.L2*c)));
                beta_neg = atan2d(-sqrt(1-((self.L2^2 + c^2 - self.L3^2)/(2*self.L2*c))^2) ,((self.L2^2 + c^2 - self.L3^2)/(2*self.L2*c)));
                % elbow positive here only
                beta = beta_pos;
                
                phi_pos = atan2d(sqrt(1- ((c^2 + r4^2 - Z4^2)/(2*c*r4))^2),((c^2 + r4^2 - Z4^2)/(2*c*r4)));
                phi_neg = atan2d(-sqrt(1- ((c^2 + r4^2 - Z4^2)/(2*c*r4))^2),((c^2 + r4^2 - Z4^2)/(2*c*r4)));
                
                % fix for being inverted around the joint2 position
                % if the joint4 Z value is above joint 2 Z value, use phi positive
                % if the joint4 Z value is below joint 2 Z value, use phi negative
                if (Z4 >= 0)
                    phi = phi_pos;
                else
                    phi = phi_neg;
                end
                
                theta_2_pos = 90 - (beta + phi + atan2d(24, 128));
                
                % theta 3
                gamma_pos = atan2d(sqrt(1 - ((self.L2^2 + self.L3^2 - c^2)/(2*self.L2*self.L3))^2),((self.L2^2 + self.L3^2 - c^2)/(2*self.L2*self.L3)));
                gamma_neg = atan2d(-sqrt(1 - ((self.L2^2 + self.L3^2 - c^2)/(2*self.L2*self.L3))^2),((self.L2^2 + self.L3^2 - c^2)/(2*self.L2*self.L3)));
                % gamma positive here only
                gamma = gamma_pos;
                
                theta_3_pos =  90 - (gamma - atan2d(24, 128));
                
                % theta 4
                theta_4_pos = alpha_EE0 - theta_2_pos - theta_3_pos;
                
            catch ER
                warning("Solution is unreachable using the elbow positive configuration " + ER.message)
                %return the current joint positions by just simply reading them
            end
            
            
            % configuration elbow negative
            try
                % Theta 2 calculations:
                beta = beta_neg;
                
                % fix for being inverted around the joint2 position
                % if the joint4 Z value is above joint 2 Z value, use phi positive
                % if the joint4 Z value is below joint 2 Z value, use phi negative
                if (Z4 >= 0)
                    phi = phi_pos;
                else
                    phi = phi_neg;
                end
                
                theta_2_neg = 90 - (beta + phi + atan2d(24, 128));
                
                % theta 3
                gamma = gamma_neg;
                
                theta_3_neg =  90 - (gamma - atan2d(24, 128));
                
                % theta 4
                theta_4_neg = alpha_EE0 - theta_2_neg - theta_3_neg;
                
            catch ER
                warning("Solution is unreachable using the elbow negative configuration " + ER.message)
                %return the current joint positions by just simply reading them
            end
            
            
            
            % check if the positive or negative configurations work with what we are trying to achieve
            % check elbow positive
            try
                if (theta_2_pos < -90 || theta_2_pos > 90)
                    error("Theta_2 positive out of range")
                end
                
                if (theta_3_pos < -100 || theta_3_pos > 90)
                    error("Theta_3 positive out of range")
                end
                
                
                if (theta_4_pos < -100 || theta_4_pos > 120)
                    error("Theta_4 positive out of range")
                end
                
                theta_2 = theta_2_pos;
                theta_3 = theta_3_pos;
                theta_4 = theta_4_pos;
                
            catch ER
                %one of the joints in this configuration is out of range
                warning("Elbow positive " + ER.message);
            end
            
            % check elbow negative
            try
                if (theta_2_neg < -90 || theta_2_neg > 90)
                    error("Theta_2 negative out of range")
                end
                
                if (theta_3_neg < -100 || theta_3_neg > 90)
                    error("Theta_3 negative out of range")
                end
                
                
                if (theta_4_neg < -100 || theta_4_neg > 120)
                    error("Theta_4 negative out of range")
                end
                
                theta_2 = theta_2_neg;
                theta_3 = theta_3_neg;
                theta_4 = theta_4_neg;
            catch ER
                %one of the joints in this configuration is out of range
                warning("Elbow negative " + ER.message);
            end
            
            % return the caluclated joint angles
            q = [theta_1 theta_2 theta_3 theta_4];
        end
        
        % Takes in 4x4 or 4x6 matrix of trajectory coefficients for each joint/point where
        %    each row represents a coefficients for one joint/point for the total time for the motion (ms)
        %    automatically switches between cubic and quintic trajectories based on the number of coefficients
        % Takes in the total time for the motion (ms)
        % Also takes a boolean isJoint if the trajectories (cubic or quintic) are for joints or coordinates
        % Moves arm from position 1 to position 2 along the trajectory
        % Returns a nx5 matrix of time and joint angle data
        function T = run_trajectory(self, trajectories, time_final, isJoint, robot_modeling)
            % Prepare data arrays
            timeData = zeros(1,1);
            velData = zeros(1,6);
            index = 1;
            
            % Check if cubic or quintic trajectory
            isCubic = (length(trajectories) == 4);
            
            % Get starting time stamps
            timeStart = posixtime(datetime('now')) * 1000; % ms
            time = posixtime(datetime('now')) * 1000 - timeStart; % ms
            
            % Loop until time is up while collecting data and moving the robot along the trajectory
            while time < time_final
                time = posixtime(datetime('now')) * 1000 - timeStart; % ms
                
                % Update the live plot model
                jakub = self.fdk3001();
                q_current = self.setpoint_js();           
                robot_modeling.update_plot(q_current, jakub);                   
                
                if(isCubic == true)
                    % Calculate current joint poses based on trajectory cubic
                    % coefficients and current time
                    % the math works for both Joint1-4 interpolation or XYZA
                    q1 = trajectories(1, 1) + time*trajectories(1, 2) + time^2*trajectories(1, 3) + time^3*trajectories(1, 4);
                    q2 = trajectories(2, 1) + time*trajectories(2, 2) + time^2*trajectories(2, 3) + time^3*trajectories(2, 4);
                    q3 = trajectories(3, 1) + time*trajectories(3, 2) + time^2*trajectories(3, 3) + time^3*trajectories(3, 4);
                    q4 = trajectories(4, 1) + time*trajectories(4, 2) + time^2*trajectories(4, 3) + time^3*trajectories(4, 4);   
                else
                    % Calculate current joint poses based on trajectory quintic
                    % coefficients and current time
                    % the math works for both Joint1-4 interpolation or XYZA
                    
                    q1 = trajectories(1, 1) + time*trajectories(1, 2) + time^2*trajectories(1, 3) + time^3*trajectories(1, 4) + time^4*trajectories(1, 5) + time^5*trajectories(1, 6);
                    q2 = trajectories(2, 1) + time*trajectories(2, 2) + time^2*trajectories(2, 3) + time^3*trajectories(2, 4) + time^4*trajectories(2, 5) + time^5*trajectories(2, 6);
                    q3 = trajectories(3, 1) + time*trajectories(3, 2) + time^2*trajectories(3, 3) + time^3*trajectories(3, 4) + time^4*trajectories(3, 5) + time^5*trajectories(3, 6);
                    q4 = trajectories(4, 1) + time*trajectories(4, 2) + time^2*trajectories(4, 3) + time^3*trajectories(4, 4) + time^4*trajectories(4, 5) + time^5*trajectories(4, 6);
                end
                 
                % Move the robot
                % also differentiate between Joint1-4 or XYZA
                if(isJoint == true)
                    self.servo_jp([q1 q2 q3 q4]);
                else
                    self.servo_jp(self.ik3001([q1, q2, q3, q4]));
                end
                
                % Collect joint and time data
                timeData(index, 1) = time;
                velData(index, :) = jakub';
                index = index + 1;

                pause(0.001);
            end
            
            % append the time and position data into a single matrix, then return it
            T = [timeData velData];
        end
        
        % Takes in 4x4 or 4x6 matrix of trajectory coefficients for each joint/point where
        %    each row represents a coefficients for one joint/point for the total time for the motion (ms)
        %    automatically switches between cubic and quintic trajectories based on the number of coefficients
        % Takes in the total time for the motion (ms)
        % Also takes a boolean isJoint if the trajectories (cubic or quintic) are for joints or coordinates
        % Moves arm from position 1 to position 2 along the trajectory
        % Returns a nx5 matrix of time and joint angle data
        function T = run_trajectory_Estop(self, trajectories, time_final, isJoint, modelO)
            % Prepare data arrays
            timeData = zeros(1,1);
            posData = zeros(1,4);
            index = 1;
            
            % Check if cubic or quintic trajectory
            isCubic = (length(trajectories) == 4);
            
            % Get starting time stamps
            timeStart = posixtime(datetime('now')) * 1000; % ms
            time = posixtime(datetime('now')) * 1000 - timeStart; % ms
            
            % Loop until time is up while collecting data and moving the robot along the trajectory
            [data, Stop] = self.E_stop();
            while time < time_final && not(Stop)
                time = posixtime(datetime('now')) * 1000 - timeStart; % ms
                
                    if(isCubic == true)
                        % Calculate current joint poses based on trajectory cubic
                        % coefficients and current time
                        % the math works for both Joint1-4 interpolation or XYZA
                        q1 = trajectories(1, 1) + time*trajectories(1, 2) + time^2*trajectories(1, 3) + time^3*trajectories(1, 4);
                        q2 = trajectories(2, 1) + time*trajectories(2, 2) + time^2*trajectories(2, 3) + time^3*trajectories(2, 4);
                        q3 = trajectories(3, 1) + time*trajectories(3, 2) + time^2*trajectories(3, 3) + time^3*trajectories(3, 4);
                        q4 = trajectories(4, 1) + time*trajectories(4, 2) + time^2*trajectories(4, 3) + time^3*trajectories(4, 4);  
                    else
                        % Calculate current joint poses based on trajectory quintic
                        % coefficients and current time
                        % the math works for both Joint1-4 interpolation or XYZA
                        
                        q1 = trajectories(1, 1) + time*trajectories(1, 2) + time^2*trajectories(1, 3) + time^3*trajectories(1, 4) + time^4*trajectories(1, 5) + time^5*trajectories(1, 6);
                        q2 = trajectories(2, 1) + time*trajectories(2, 2) + time^2*trajectories(2, 3) + time^3*trajectories(2, 4) + time^4*trajectories(2, 5) + time^5*trajectories(2, 6);
                        q3 = trajectories(3, 1) + time*trajectories(3, 2) + time^2*trajectories(3, 3) + time^3*trajectories(3, 4) + time^4*trajectories(3, 5) + time^5*trajectories(3, 6);
                        q4 = trajectories(4, 1) + time*trajectories(4, 2) + time^2*trajectories(4, 3) + time^3*trajectories(4, 4) + time^4*trajectories(4, 5) + time^5*trajectories(4, 6);
                    end
                
                
                % Move the robot
                % also differentiate between Joint1-4 or XYZA
                if(isJoint == true)
                    self.servo_jp([q1 q2 q3 q4]);
                else
                    self.servo_jp(self.ik3001([q1, q2, q3, q4]));
                end
                
                % model updating
                modelO.update_plot(self.setpoint_js(), self.fdk3001());

                % Collect joint and time data
                timeData(index, 1) = time;
                posData(index, :) = self.setpoint_js();
                [detData(index, :), Stop] = self.E_stop();
                index = index + 1;
                pause(0.001); % small pause for updating live plot
            end
            
            % change the text to singularity if
            if Stop
                set(modelO.ee_text_ref, 'String',"Singularity!"); 
            end

            % append the time and position data into a single matrix, then return it
            T = [timeData posData detData];
        end
        
        % Extra credit from lab 3 to have robot move in circular trajectory
        % Takes in time for movement and trajectory planner object
        % Returns time and position data of the movement
        function T = runCircle(self, time_final, t_planner)
            % Prepare data arrays
            timeData = zeros(1,1);
            posData = zeros(1,4);
            index = 1;
            
            % prepare the parametrized 3D circle
            circleCenterPoint = [190, 30, 150];
            unitVector1 = [0.5774 0.5774 0.5774];
            unitVector2 = [0.4083 -0.8165 0.4083];
            radius = 80;
            
            s_trajectory = t_planner.quintic_traj(0, time_final, 0, 2*pi, 0, 0, 0, 0);  % the s parameter quintic interpolation coeffs
            
            A = 40; % the alpha angle
            
            % Get starting time stamps
            timeStart = posixtime(datetime('now')) * 1000; % ms
            time = posixtime(datetime('now')) * 1000 - timeStart; % ms
            
            while time < time_final
                time = posixtime(datetime('now')) * 1000 - timeStart; % ms
                
                % Calculate current joint poses based on quintic parameter s interpolation
                % and a 3D algebraic parametrized equation of a circle
                s_param = s_trajectory(1) + time*s_trajectory(2) + time^2*s_trajectory(3) + time^3*s_trajectory(4) + time^4*s_trajectory(5) + time^5*s_trajectory(6);
                
                X = circleCenterPoint(1) + radius*cos(s_param)*unitVector1(1) + radius*sin(s_param)*unitVector2(1);
                Y = circleCenterPoint(2) + radius*cos(s_param)*unitVector1(2) + radius*sin(s_param)*unitVector2(2);
                Z = circleCenterPoint(3) + radius*cos(s_param)*unitVector1(3) + radius*sin(s_param)*unitVector2(3);
                
                % Move the robot
                self.servo_jp(self.ik3001([X, Y, Z, A]));
                
                % Get joint and time data
                timeData(index, 1) = time;
                posData(index, :) = self.setpoint_js();
                index = index + 1;
            end
            
            T = [timeData posData];
        end
        
        % Takes the current joint angles of the robot in degrees and
        % returns the Jacobian [6 x 4] of the robot in that configuration 
        % return LINEAR velocites and RADIAN angular velocities
        function J = jakub3001(self, q)
            % convert joints from deg to rad
            theta1 = deg2rad(q(1));
            theta2 = deg2rad(q(2)) - acos(24 / self.L2);   %theta2 offset
            theta3 = deg2rad(q(3)) + acos(24 / self.L2);  %theta3 offset
            theta4 = deg2rad(q(4));
            
            % hardcoded Jacobian matrix calculated using the symbolic toolbox
            J = [(667*cos(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/5 + (667*sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/5 + 124*sin(theta1)*sin(theta2)*sin(theta3) - 8*265^(1/2)*cos(theta2)*sin(theta1) - 124*cos(theta2)*cos(theta3)*sin(theta1), (667*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/5 - (667*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/5 - 8*265^(1/2)*cos(theta1)*sin(theta2) - 124*cos(theta1)*cos(theta2)*sin(theta3) - 124*cos(theta1)*cos(theta3)*sin(theta2), (667*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/5 - (667*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/5 - 124*cos(theta1)*cos(theta2)*sin(theta3) - 124*cos(theta1)*cos(theta3)*sin(theta2), (667*sin(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/5 - (667*cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/5;
                 8*265^(1/2)*cos(theta1)*cos(theta2) - (667*sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))/5 - 124*cos(theta1)*sin(theta2)*sin(theta3) - (667*cos(theta4)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)))/5 + 124*cos(theta1)*cos(theta2)*cos(theta3), (667*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/5 - (667*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/5 - 124*cos(theta2)*sin(theta1)*sin(theta3) - 124*cos(theta3)*sin(theta1)*sin(theta2) - 8*265^(1/2)*sin(theta1)*sin(theta2), (667*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/5 - (667*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/5 - 124*cos(theta2)*sin(theta1)*sin(theta3) - 124*cos(theta3)*sin(theta1)*sin(theta2), (667*sin(theta4)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)))/5 - (667*cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))/5;
                 0, 124*sin(theta2)*sin(theta3) - 124*cos(theta2)*cos(theta3) - 8*265^(1/2)*cos(theta2) - (667*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/5 + (667*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/5, 124*sin(theta2)*sin(theta3) - 124*cos(theta2)*cos(theta3) - (667*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/5 + (667*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/5, (667*sin(theta4)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)))/5 - (667*cos(theta4)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)))/5;
                 0, -sin(theta1), -sin(theta1), -sin(theta1);
                 0, cos(theta1), cos(theta1), cos(theta1);
                 1, 0, 0, 0];

           J = deg2rad(J); % Convert to rad^2 to fix weird unit multiplication we had somewhere (Jakub said I can credit him with this)

        end

        % Takes current joint angles and current joint velocities at
        % run time
        % Returns a 6x1 vector with the task-space linear and angular
        % velocties of the end effector
        function pdot = fdk3001(self)
            % get q and qdot from robot
            robot_data = self.measured_js(true, true);
            q = robot_data(1, :);
            qdot = robot_data(2, :)';

            % get jacobian
            jacobian = self.jakub3001(q);

            % multiply the jacobian and qdot to get pdot
            pdot = jacobian*qdot;
        end
        
        % Function for returning whether we've met e-stop condition
        % Returns determinant value for data collection and stop boolean
        % for whether to e-stop
        function [det_val, stop] = E_stop(self)            
            matrix = self.jakub3001(self.setpoint_js()); % Current Jacobian
            det_val = det(matrix(1:3,1:3));             
            stop = false;
            
            % Check if we need to e-stop
            if det_val <= 0.5
                stop = true;
            end
        end
        
        % Takes in x, y, z, alpha values as 1x4 array, and takes in mm
        % offset for z value and degree offset for alpha
        % Returns array with adjusted z and alpha values
        function new_q = appro_za(self, q, z_offset, alpha_offset)
            new_q = [q(1), q(2), q(3)+z_offset, q(4)+alpha_offset];
        end
        
    end % end methods
end % end class