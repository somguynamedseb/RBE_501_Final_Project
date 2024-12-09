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
        s1;
        s2;
        s3;
        s4;
        Slist;
        M;
        dt; 
        
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
            
            % Set end goal joint positions
            self.goalJS = [0, 0, 0, 0];
            
            % Set up link lengths in m
            self.L0 = 0.096326;
            self.L1 = 0.128;
            self.L2 = 0.024;
            self.L3 = 0.124;
            self.L4 = 0.1334;

            % Set universal step size for velocity control in seconds
            self.dt = 0.22;

            % Jakub is a chill dude (define Jacobians)
            self.s1 = [0,0,1,0,0,0].';
            self.s2 = [0,1,0,-self.L0,0,0].';
            self.s3 = [0,1,0,-self.L0-self.L1,0,self.L2].';
            self.s4 = [0,1,0,-self.L0-self.L1,0,self.L2+self.L3].';
            self.Slist = [self.s1,self.s2,self.s3,self.s4];

            self.b1 = [1,0,0,0,self.L2+self.L3+self.L4,0].';
            self.b2 = [0,1,0,-self.L2-self.L3-self.L4,0,-self.L1].';
            self.b3 = [0,1,0,-self.L3-self.L4,0,0].';
            self.b4 = [0,1,0,-self.L4,0,0].';
            self.Blist = [self.b1,self.b2,self.b3,self.b4];
            
            % Define home position
            self.M = [[0,0,-1,self.L2+self.L3+self.L4];
                [0,1,0,0];
                [1,0,0,self.L0+self.L1];
                [0,0,0,1]];
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
        
        
        
        
        %%
        
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

            % pause to let robot move
            pause(interTime/1000);
        end
        
        % Gets the joint positions and velocities and returns what the user
        % wants of those data. Data that is not requested is returned as 0s
        % GETPOS [boolean] - True if user wants position data, false
        % otherwise
        % GETVEL [boolean] - True if user wants velocity data, false
        % otherwise
        function fast_readings = measured_js(self, GETPOS, GETVEL, GETCUR)
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

            if GETCUR
                 fast_readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
            end
        end

        % Outputs a 1x4 vector of joint angles to reach the desired position
        % theta_1 limits -90, 90
        % theta_2 limits -90, 90
        % theta_3 limits -100, 90
        % theta_4 limits -100, 120
        function [q,s] = ikspace(self,target)
            initial_guess = [0;0;0;0];
            eomg = 0.001;
            ev = 0.001;
            [q,s] = IKinSpace(self.Slist,self.M,target,initial_guess,eomg,ev);
            % return the caluclated joint angles
            % q = [theta_1 theta_2 theta_3 theta_4];
        end
        
        % Creates a smooth curve with ramp up and ramp down between two
        % points over a given time.
        % INPUTS: 4x4 T matrices for initial and target pos, target time
        % in seconds, and where the stitch will occur (0 for none, 1 for start, or
        % 2 for end)
        % OUTPUTS: nx6 arrays of positions, velocities, and accelerations
        function [pos_list,vel_list,acc_list] = LSPBUpdated(self,initial_pos,target_pos,target_time,stitch)
            % Convert orientation to quaternion for interpolation
            initial_orientation = rotm2eul(initial_pos(1:3,1:3))';
            target_orientation = rotm2eul(target_pos(1:3,1:3))';

            % Set up calculations
            t = 0;
            buffer_percent = 0.2;
            buffer_time = target_time * buffer_percent;
            distance =  (initial_pos(1:3,4)-target_pos);
            target_vel = distance./((1-buffer_percent) * target_time);
            target_acc = target_vel./buffer_time;
            current_pos = [initial_orientation;initial_pos(1:3,4)];
            current_vel = [0;0;0;0;0;0]; % structured [w; v]
            current_acc = [0;0;0;0;0;0]; 
            pos_list = [];
            vel_list = [];
            acc_list = [];
            iterations = target_time/self.dt;

            % Recursive calculation
            for i = 1:iterations
                t = i*self.dt;

                % Beginning buffer (if applicable)
                if (time < buffer_time && stitch ~= 1)
                    current_acc(:) = target_acc;
                    current_vel = current_vel + (current_acc.*self.dt);
                    current_pos = current_pos + (current_vel.*self.dt);
                % Ending buffer (if applicable)
                elseif (time > buffer_time && stitch ~= 2)
                    current_acc(:) = -target_acc;
                    current_vel = current_vel + (current_acc.*self.dt);
                    current_pos = current_pos + (current_vel.*self.dt);
                % Linear interpolation
                else
                    current_acc(:) = 0;
                    current_vel = current_vel + (current_acc.*self.dt);
                    current_pos = current_pos + (current_vel.*self.dt);
                end

                % Record data
                pos_arr(i,1:6) = current_pos';
                vel_arr(i,1:6) = current_vel';
                acc_arr(i,1:6) = current_acc';
            end
        end 

        % Converts cartesian and rotational velocities to joint velocities
        % INPUTS: Both an nx6 array of positions and velocitiesoutput by
        % LSPB
        % OUTPUTS: An nx4 array of joint velocities
        function [qdot_list] = LSPBtoVelUpdated(self,pos_list,vel_list)
            for i = 1:size(vel_list)
                % Compute joint angles from end effector position and euler
                % angle orientation
                x = pos_list(i,1);
                y = pos_list(i,2);
                z = pos_list(i,3);
                R = eul2rotm(pos_list(i,4:6));
                T04 = [[0,0,0,x];
                        [0,0,0,y];
                        [0,0,0,z];
                        [0,0,0,1]];
                T04(1:3,1:3) = R;
                [q,s] = self.ikspace(T04);
                
                % Compute Body Jacobian at current pose
                Jb = JacobianBody(self.Blist,q);

                % Calculate joint velocities
                end_effector_vel = vel_list(i,:)';
                qdot = pinv(Jb)*end_effector_vel;

                % Store data
                qdot_list(i,1:4) = qdot';
            end
        end

        
        function [pos_arr,vel_arr,acc_arr,z_arr] = LSPB(self,initial_pos,target_pos,target_time)

            buffer_percent = 0.2;
            buffer_time = target_time * buffer_percent;
            initial_pos = initial_pos.'
            target_pos = target_pos.'
            current_state = [initial_pos,0,0,0,0,0,0]; %x,y,z,vx,vy,vz,ax,ay,az
            pos_arr = [];
            vel_arr = [];
            acc_arr = [];

            dist =  (current_state(1:3)-target_pos);
            target_vel = dist/((1-buffer_percent) * target_time);
            target_acc = target_vel/buffer_time;
            iterations = target_time/self.dt;

            for i = 0:iterations
                time = i*self.dt;
            
                if time< buffer_time   
                    current_state(7:9) = target_acc;
                    current_state(4:6) = current_state(4:6) + current_state(7:9) * self.dt;
                    current_state(1:3) = current_state(1:3) + current_state(4:6) * self.dt;
                elseif time> target_time - buffer_time   
                    current_state(7:9) = -target_acc;
                    current_state(4:6) = current_state(4:6) + current_state(7:9) * self.dt;
                    current_state(1:3) = current_state(1:3) + current_state(4:6) * self.dt;
                else
                    current_state(7:9) = 0;
                    current_state(4:6) = current_state(4:6);
                    current_state(1:3) = current_state(1:3) + current_state(4:6) * self.dt;
                end
               
               z_arr(i+1) = atan2(current_state(2),current_state(1))
               pos_arr(i+1,1:3) = (current_state(1:3));
               vel_arr(i+1,1:3) = [current_state(4:6)];
               acc_arr(i+1,1:3) = (current_state(7:9));
            end
            % plot(vel_arr)

        end
        
        function [qdot_arr] = LSPBtoVel3(self,pos_arr,vel_arr,z_arr)
                x = pos_arr(1,1);
                y = pos_arr(1,2);
                z = pos_arr(1,3);
                x_tar = pos_arr(end-1,1);
                y_tar = pos_arr(end-1,2);
                z_tar = pos_arr(end-1,3); 
                
                theta1 = atan2(y,x);
                T04_1 =[[0,-sin(theta1),-cos(theta1),x];
                            [0,cos(theta1),-sin(theta1),y];
                            [1,0,0,z];
                            [0,0,0,1]];

                theta1 = atan2(y_tar,x_tar);
                T04_2 = [[0,-sin(theta1),-cos(theta1),x];
                            [0,cos(theta1),-sin(theta1),y];
                            [1,0,0,z];
                            [0,0,0,1]];
                  
                    
                q = self.ikspace(T04_1);
                q_tar = self.ikspace(T04_2);
                
                q_state = q;
                q_vels = [0,0,0,0];


                buffer_percent = 0.2;
                buffer_time = target_time * buffer_percent;
                vel_arr = [];
                acc_arr = [];
    
                dist =  (q-q_tar);
                target_vel = dist/((1-buffer_percent) * target_time);
                target_acc = target_vel/buffer_time;
                iterations = target_time/self.dt;


                
                for i = 0:iterations
                    time = i*self.dt;
                
                    if time< buffer_time   
                        q_vels = q_vels + target_acc * self.dt;
                        q_state = q_state + q_vels * self.dt;
                    elseif time> target_time - buffer_time   
                        q_vels = q_vels - target_acc * self.dt;
                        q_state = q_state + q_vels * self.dt;
                    else
                        q_vels = q_vels;
                        q_state = q_state + q_vels * self.dt
                    end
               
                   z_arr(i+1) = atan2(current_state(2),current_state(1));
                   pos_arr(i+1,1:4) = (q_state));
                   vel_arr(i+1,1:4) = (q_vels));
                end
                joint1_vel = (q(1)-q_tar(1))/target_time;
                pos_arr(:,1) = joint_vel;
    
                    
                end
            end
        


            % jacobian = [
                    %     0 0 0 0;
                    %     0 1 1 1;
                    %     1 0 0 0;
                    %     0 0.128 0 0;
                    %     0.28 0 0 0;
                    %     0 -0.28 -0.25 -0.13;
                    % ]
                    % 
                    % 
                    % jac_vel = jacobian(4:6, :);
                    % 
                    % vel = [vel_arr(i,1:3)]';
                    % jb = JacobianBody(jacobian,q)
                    % jbslice = jb(4:6,:)
                    % jbp= pinv(jbslice)
                    % qdot_arr(i,1:4) = (jbp*-vel);
                    % qdot_arr(i,1:4) = pinv(jac_vel)*vel_arr(i, 1:3)';
                    % q = q+((qdot_arr(i,1:4))'*self.dt);
        
    end % end methods
end % end class