%% copied the 3D plot for testing
%% independently verifies the 3D Inverse Kinematics

close all
clear

home = [281.4; 0; 224.3; 0];
pos1 = [173.6898; 48.2589; 09.1028; 80];
pos2 = [254; 71; 142; 0];
xyzp1 = [281.4 0 224.3254 0];
xyzp2 = [250 75 50 20];
xyzp3 = [200 -30 50 20];


xyzp2 = xyzp1;
xyzp1 = xyzp3;
ik3001(xyzp3)

num_interp = 50;
X = linspace(xyzp1(1), xyzp2(1), num_interp);
Y = linspace(xyzp1(2), xyzp2(2), num_interp);
Z = linspace(xyzp1(3), xyzp2(3), num_interp);
A = linspace(xyzp1(4), xyzp2(4), num_interp);

for i = 1:num_interp
    [figure_ref, frames, text_reference] = plot_arm(ik3001([X(i); Y(i); Z(i); A(i)]));
    pause(0.1);
end



% Test();

function q = ik3001(EE)

    Xe = EE(1);
    Ye = EE(2);
    Ze = EE(3);
    alpha_EE0 = EE(4);
    
    L0 = 36.076;
    L1 = 96.326 - L0;
    L2 = sqrt(128^2 + 24^2);
    L3 = 124;
    L4 = 133.4;
     
    % Coordinates to joint 2 are known
    Z2 = L0 + L1;
    
    try
        % Theta 1 is angle from x axis to (x, y) of EE
        re = sqrt(Xe^2 +Ye^2);
        theta_1_pos = atan2d(sqrt(1-(Xe/re)^2) , (Xe/re));
        theta_1_neg = atan2d(-sqrt(1-(Xe/re)^2) , (Xe/re));
        % theta_1_neg will always make the arm point the wrong direction
        if (Ye >= 0)
            theta_1 = theta_1_pos;
        else
            theta_1 = theta_1_neg;
        end
              
        if(theta_1 > 90 || theta_1 < -90)
            error("Theta_1 out of joint allowed range")
        end

    catch
        % an error occured, read and return the current joint values
        % exit the function early
    end
    
    % configuration elbow positive
    try
        % Theta 2
        Z4 = L4 * sind(alpha_EE0) - Z2 + Ze;
        r4 = re - L4 * cosd(alpha_EE0);
        c = sqrt(r4^2 +Z4^2);
        
        beta_pos = atan2d(sqrt(1-((L2^2 + c^2 - L3^2)/(2*L2*c))^2) ,((L2^2 + c^2 - L3^2)/(2*L2*c)));
        beta_neg = atan2d(-sqrt(1-((L2^2 + c^2 - L3^2)/(2*L2*c))^2) ,((L2^2 + c^2 - L3^2)/(2*L2*c)));
        beta = beta_pos;
        
        phi_pos = atan2d(sqrt(1- ((c^2 + r4^2 - Z4^2)/(2*c*r4))^2),((c^2 + r4^2 - Z4^2)/(2*c*r4)));
        phi_neg = atan2d(-sqrt(1- ((c^2 + r4^2 - Z4^2)/(2*c*r4))^2),((c^2 + r4^2 - Z4^2)/(2*c*r4)));
        
        % fix for being inverted around the join2 position
        if (Z4 >= 0)
            phi = phi_pos;
        else
            phi = phi_neg;
        end           
        
        theta_2_pos = 90 - (beta + phi + atan2d(24, 128));
        
        % theta 3
        gamma_pos = atan2d(sqrt(1 - ((L2^2 + L3^2 - c^2)/(2*L2*L3))^2),((L2^2 + L3^2 - c^2)/(2*L2*L3)));
        gamma_neg = atan2d(-sqrt(1 - ((L2^2 + L3^2 - c^2)/(2*L2*L3))^2),((L2^2 + L3^2 - c^2)/(2*L2*L3)));
        gamma = gamma_pos;

        theta_3_pos =  90 - (gamma - atan2d(24, 128));

        % theta 4
        theta_4_pos = alpha_EE0 - theta_2_pos - theta_3_pos;
    catch           
        warning("Solution is unreachable using the elbow positive configuration")
        %return the current joint positions by just simply reading them  
    end


    % configuration elbow negative
    try
        % Theta 2
        beta = beta_neg;

        % fix for being inverted around the join2 position
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
    catch           
        warning("Solution is unreachable using the elbow negative configuration")
        %return the current joint positions by just simply reading them  
    end


    % check if the positive or negative configurations work with what we
    % are trying to achieve
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
   
    
    q = [theta_1 theta_2 theta_3 theta_4];
    % theta_1 limits -90, 90
    % theta_2 limits -90, 90
    % theta_3 limits -100, 90
    % theta_4 limits -100, 120
end

function T = fk(q)
    % Get link lengths in mm
    L0 = 36.076;
    L1 = 96.326 - L0;
    L2 = sqrt(128^2 + 24^2);
    L3 = 124;
    L4 = 133.4;

    % Get the DH parameter table for the arm
    DH = [q(1) L1 0 -90;
          q(2)-acosd(24 / L2) 0 L2 0;
          q(3)+acosd(24 / L2) 0 L3 0;
          q(4) 0 L4 0];

    % Get T01
    T = dh2mat([0 L0 0 0]);   %SELF!!!       

    % Get T02
    T = T * dh2mat(DH(1,:));   %SELF!!!       

    % Get T03
    T = T * dh2mat(DH(2,:));   %SELF!!!       

    % Get T04
    T = T * dh2mat(DH(3,:));   %SELF!!!       

    % Get T05
    T = T * dh2mat(DH(4,:));   %SELF!!!     
end

function fig_ref = Test(q)
    %% tested maximum joint values
    n_points = 20;
    joint1 = linspace(-90, 90, n_points);
    joint2 = linspace(-90, 90, n_points);
    joint3 = linspace(-100, 90, n_points);
    joint4 = linspace(-100, 120, n_points);
    
    GigaArray = zeros(n_points ^ 4, 3);
    index = 1;

    % generate all the possible positions of the robot
    for j1 = joint1
        for j2 = joint2
            for j3 = joint3
                for j4 = joint4
                    Temp = fk([j1, j2, j3, j4]);
                    GigaArray(index,:) = Temp(1:3,4);
                    index = index + 1;
                end
            end
        end
    end   

    % find a boundary with shrink factor 1
    % of all the possible reachable points
    % note: takes about 2mins to graph
   % figure("Name","Workspace");    
    k = boundary(GigaArray,1);
    fig_ref = trisurf(k,GigaArray(:,1),GigaArray(:,2),GigaArray(:,3), ...
        'Facecolor','red', 'FaceAlpha',0.15,'LineWidth', 0.05);
    
    view([(-37.5 + 160) 25])
    axis equal;
    xlabel("X Axis (mm)");
    ylabel("Y Axis (mm)");
    zlabel("Z Axis (mm)");
    title("HALL 4000 Workspace");

end

%%-----------------------------
% Takes a 1x4 vector q representing the joing angle values
% Graphs the stick figure of the robot arm
% Returns a reference to the plot window
function [fig, FrameQuivers, text_ref] = plot_arm(q)     

    % Get link lengths in mm
    L0 = 36.076;
    L1 = 96.326 - L0;
    L2 = sqrt(128^2 + 24^2);
    L3 = 124;
    L4 = 133.4;

    % Get the DH parameter table for the arm
    DH = [q(1) L1 0 -90;
          q(2)-acosd(24 / L2) 0 L2 0;
          q(3)+acosd(24 / L2) 0 L3 0;
          q(4) 0 L4 0];
  

    % initialize variable to store all the T0n transformations
    % 4x4 matrix, 6 times
    FramesRaw = zeros(4, 4, 6);
    FrameQuivers = zeros(3, 6);

    %-----------
    % Get T00
    FramesRaw(:, :, 1) = eye(4);

    % Get T01
    T01 = dh2mat([0 L0 0 0]);   %SELF!!!   
    FramesRaw(:, :, 2) = T01;

    % Get T02
    T02 = T01 * dh2mat(DH(1,:));   %SELF!!!   
    FramesRaw(:, :, 3) = T02;

    % Get T03
    T03 = T02 * dh2mat(DH(2,:));   %SELF!!!   
    FramesRaw(:, :, 4) = T03;

    % Get T04
    T04 = T03 * dh2mat(DH(3,:));   %SELF!!!   
    FramesRaw(:, :, 5) = T04;

    % Get T05
    T05 = T04 * dh2mat(DH(4,:));   %SELF!!!   
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
        FrameQuivers(:, index) = FrameQuiver3D(FramesRaw(:, :, index), 60);
    end
    
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


function update_plot(q, fig, frames_ref, textref)
    
    % Get link lengths in mm
    L0 = 36.076;
    L1 = 96.326 - L0;
    L2 = sqrt(128^2 + 24^2);
    L3 = 124;
    L4 = 133.4;

    % Get the DH parameter table for the arm
    DH = [q(1) L1 0 -90;
          q(2)-acosd(24 / L2) 0 L2 0;
          q(3)+acosd(24 / L2) 0 L3 0;
          q(4) 0 L4 0];
  

    % initialize variable to store all the T0n transformations
    % 4x4 matrix, 6 times
    FramesRaw = zeros(4, 4, 6);

    %-----------
    % Get T00
    FramesRaw(:, :, 1) = eye(4);

    % Get T01
    T01 = dh2mat([0 L0 0 0]);   %SELF!!!   
    FramesRaw(:, :, 2) = T01;

    % Get T02
    T02 = T01 * dh2mat(DH(1,:));   %SELF!!!   
    FramesRaw(:, :, 3) = T02;

    % Get T03
    T03 = T02 * dh2mat(DH(2,:));   %SELF!!!   
    FramesRaw(:, :, 4) = T03;

    % Get T04
    T04 = T03 * dh2mat(DH(3,:));   %SELF!!!   
    FramesRaw(:, :, 5) = T04;

    % Get T05
    T05 = T04 * dh2mat(DH(4,:));   %SELF!!!   
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
    
    %update the data for the 4-dof arm
    set(fig, 'XData', X2Graph, 'YData', Y2Graph, 'ZData', Z2Graph);
    
    set(textref,'Position', [X2Graph(end) Y2Graph(end) Z2Graph(end)]);
    set(textref, 'String',[' (', ...
        (num2str(round(X2Graph(end), 4), 4)), ', ', ...
        (num2str(round(Y2Graph(end), 4), 4)), ', ', ...
        (num2str(round(Z2Graph(end), 4), 4)), ') ']);                     
%             self.ee_text_ref = text(X2Graph(end), Y2Graph(end), Z2Graph(end), [' (', ...
%                 (num2str(round(X2Graph(end), 4), 4)), ', ', ...
%                 (num2str(round(Y2Graph(end), 4), 4)), ', ', ...
%                 (num2str(round(Z2Graph(end), 4), 4)), ') ']);

    for joint_index = 1:6                
        %set the quiver data by parsing in the current transformation matrix    
        x_coord = FramesRaw(1, 4, joint_index);
        y_coord = FramesRaw(2, 4, joint_index);
        z_coord = FramesRaw(3, 4, joint_index);

        for quiver_index = 1:3     
            %unit vector of X1-axis, then Y1, then Z1 axis
            U_coord = FramesRaw(1, quiver_index, joint_index);
            V_coord = FramesRaw(2, quiver_index, joint_index); 
            W_coord = FramesRaw(3, quiver_index, joint_index);
 
            %quiver3(x_coord, y_coord, z_coord, U_coord, V_coord, W_coord, scale, color);                         
            set(frames_ref(quiver_index, joint_index), 'XData', x_coord, 'YData', ...
                y_coord, 'ZData', z_coord, 'UData', U_coord, ...
                'VData', V_coord, 'WData', W_coord); 
        end
    end
end


function T = dh2mat(q) %SELF!!!
        T = [cosd(q(1)) -sind(q(1))*cosd(q(4)) sind(q(1))*sind(q(4)) q(3)*cosd(q(1));
             sind(q(1)) cosd(q(1))*cosd(q(4)) -cosd(q(1))*sind(q(4)) q(3)*sind(q(1));
             0 sind(q(4)) cosd(q(4)) q(2);
             0 0 0 1];
end


% takes in a 4x4 Transformation Matrix
% Outputs a 3x6 matrix representing the 3 Quiver3D of the frame X, Y, and Z
%   also directly plots the quivers
function Quiv = FrameQuiver3D(TransMatrix, scale)        
            Quiv = zeros(3, 1);
            x_coord = TransMatrix(1, 4);
            y_coord = TransMatrix(2, 4);
            z_coord = TransMatrix(3, 4);
        
            for idx = 1:3
                %unit vector of X1-axis, then Y1, then Z1 axis
                U_coord = TransMatrix(1, idx);
                V_coord = TransMatrix(2, idx);  
                W_coord = TransMatrix(3, idx);

%                 rnd_colorR = rand(1);
%                 rnd_colorG = rand(1);
%                 rnd_colorB = rand(1);

                if idx == 3
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, ...
                        V_coord, W_coord, scale, Color='b');
                elseif idx == 2            
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, ...
                        V_coord, W_coord, scale, Color = 'g', LineStyle="-.");    %green
                else                    % [rnd_colorR, rnd_colorG, rnd_colorB],
                    Quiv(idx) = quiver3(x_coord, y_coord, z_coord, U_coord, ...
                        V_coord, W_coord, scale,  Color = 'r', LineStyle="-.");    %red
                end              
            end
        end