classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
        cam_TForm   % Camera Rigid 3D TForm
        cam_gridPoints; % Camera XY grid points in mm
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
            self.find_grid_points(); % update the grid points automatically         
        end

        function tForm = getTForm(self)
            tForm = self.cam_TForm;
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraInstrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        function cat_P = getGridPoints(self)
            cat_P = self.cam_gridPoints;
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                CamCali; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            self.cam_TForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end
        
        % Takes in x, y pixel coordinates of a point in the camera image
        % Returns the x, y values of that point in the checkboard
        function point_world = cam_to_checker(self, px)
            % Camera rotation and translation matrices
            R = self.cam_pose(1:3, 1:3);
            t = self.cam_pose(1:3, 4);
            
            point_world = pointsToWorld(self.getCameraInstrinsics(), R, t, px);
        end
        
        % Takes in a point on the checkerboard [x, y]
        % Returns the x, y, z for the robot to go to to reach that point
        % Z coordinate is 0, but when moving the robot, it should be
        % greater than 0 to prevent oopsies
        function point_robot = checker_to_robot(self, point_world)
            % Get the transformation from robot F0 to checkerboard
            R_0_checker = [ 0  1  0; 1  0  0; 0  0 -1];
            t_0_checker = [80; -113; 0];
            t_0_correction_offset = [25; 25; 0];
            T_0_check = [R_0_checker, t_0_checker + t_0_correction_offset; zeros(1,3), 1];

            point_robot = inv(T_0_check) * [point_world'; 0; 1];
        end
        
        % Finds and returns the x, y pixel coordinates of the grid post-fisheye
        % removal. Also verifies that we saw the whole checkerboard
        function gridPoints = find_grid_points(self) 
            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            
            % Check that we got the whole checkerboard, and quit if not
            gridPoints = imagePoints;
            self.cam_gridPoints = gridPoints;
            if size(gridPoints(:, 1)) ~= 40
               error ("Unable to detect full checkerboard");
            end           
        end

        % uses camera pose and instrinsics to find the checkerboard corners
        % returns the corners in the image in pixel coordinates
        function corners = findCheckerboardCorners(self)
            R = self.getCameraPose();
            R = R(1:3, 1:3);
            T = self.getCameraPose();
            T = T(1:3, 4);
            
            offset_in_mm = 35;
            square_size_mm = 25;

            TL = worldToImage(self.getCameraInstrinsics, R, T, [0 - offset_in_mm, 0 - offset_in_mm, 0]);
            TR = worldToImage(self.getCameraInstrinsics, R, T, [9*square_size_mm + offset_in_mm, 0 - offset_in_mm, 0]);
            BL = worldToImage(self.getCameraInstrinsics, R, T, [0 - offset_in_mm, 3*square_size_mm + offset_in_mm, 0]);
            BR = worldToImage(self.getCameraInstrinsics, R, T, [9*square_size_mm + offset_in_mm, 3*square_size_mm + offset_in_mm, 0]);

            corners = [TL; TR; BR; BL];
        end

        % acquires the hsv_image of the cropped region of interest
        % and takes in a boolean if we want to display graph
        % returns the balls' centers as six nx2 arrays
        % where the order is blue, yellow, red, green, gray, orange
        function [bCent, yCent, rCent, gCent, grCent, oCent] = getBalls(self, doGraph)
            % Get HSV image of current workspace
            raw_img = self.getImage(); 
            hsv_img = rgb2hsv(raw_img);     
            
            % Create empty mask of the correct size
            BW = false(size(raw_img,1),size(raw_img,2));
            
            % cutoff the image only to the checkerboard
            corners = self.findCheckerboardCorners();
            
            % get the image mask size
            m = size(BW, 1);
            n = size(BW, 2);
            
            % create a quad-polygon and mask the whole image
            addedRegion = poly2mask(corners(:, 1), corners(:, 2), m, n);
            BW = BW | addedRegion;
            
            % Set background pixels where BW is false to zero.
            raw_img(repmat(~BW,[1 1 3])) = 0;
            hsv_img(repmat(~BW,[1 1 3])) = 0;
            
            % Show raw image if wanted
            if(doGraph)
                f_raw = figure;
                imshow(raw_img);
                title("Raw Image");
                drawnow;
                pause(0.1);
            end

            % blue, yellow, red, green, gray, orange blobs collection
            [blueCenters, bluedRadii] = self.blobImageCenter(raw_img, hsv_img, "blue", 1);
            [yellowCenters, yellowdRadii] = self.blobImageCenter(raw_img, hsv_img, "yellow", 1);
            [redCenters, reddRadii] = self.blobImageCenter(raw_img, hsv_img, "red", 1);
            [greenCenters, greendRadii] = self.blobImageCenter(raw_img, hsv_img, "green", 1);
            [grayCenters, graydRadii] = self.blobImageCenter(raw_img, hsv_img, "gray", 1);
            [orangeCenters, orangedRadii] = self.blobImageCenter(raw_img, hsv_img, "orange", 1);            
            
            % Show image after each classification process (if wanted)
            if(doGraph)
                % full image overlayed with all the balls, centers, XY, radii,
                % in the appropriate colors
                %f_final = figure; imshow(raw_img);          
                
                hold on
                % if there are any blue centers, plot them
                if ~isempty(blueCenters)
                    viscircles(blueCenters, bluedRadii, 'Color', 'b');
                    plot(blueCenters(:,1),blueCenters(:,2),'k*');
                end
    
                % if there are any yellow centers, plot them
                if ~isempty(yellowCenters)
                    viscircles(yellowCenters, yellowdRadii, 'Color', 'y');
                    plot(yellowCenters(:,1),yellowCenters(:,2),'k*');
                end
    
                % if there are any red centers, plot them
                if ~isempty(redCenters)
                    viscircles(redCenters, reddRadii, 'Color', 'r');
                    plot(redCenters(:,1),redCenters(:,2),'k*');
                end
    
                % if there are any green centers, plot them
                if ~isempty(greenCenters)
                    viscircles(greenCenters, greendRadii, 'Color', 'g');
                    plot(greenCenters(:,1),greenCenters(:,2),'k*');
                end
    
                % if there are any gray centers, plot them
                if ~isempty(grayCenters)
                    viscircles(grayCenters, graydRadii, 'Color', 'k');
                    plot(grayCenters(:,1),grayCenters(:,2),'w*');
                end
    
                % if there are any orange centers, plot them
                if ~isempty(orangeCenters)
                    viscircles(orangeCenters, orangedRadii, 'Color', 'm');
                    plot(orangeCenters(:,1),orangeCenters(:,2),'k*');
                end
           
                hold off
                title("Fully Classified");
                drawnow;
                pause(0.1);   
            end
            
            % assign all the return values
            bCent = blueCenters;
            yCent = yellowCenters;
            rCent = redCenters;
            gCent = greenCenters;
            grCent = grayCenters;
            oCent = orangeCenters;
        end

        % Specifically looks for a green ball for live tracking. Returns
        % the coordinates of the center of the green ball if found
        function gCent = getGreenBalls(self)
            % Get HSV image of current workspace
            raw_img = self.getImage(); 
            hsv_img = rgb2hsv(raw_img);     
            
            % Create empty mask of the correct size
            BW = false(size(raw_img,1),size(raw_img,2));
            
            % cutoff the image only to the checkerboard
            corners = self.findCheckerboardCorners();
            
            % get the image mask size
            m = size(BW, 1);
            n = size(BW, 2);
            
            % create a quad-polygon and mask the whole image
            addedRegion = poly2mask(corners(:, 1), corners(:, 2), m, n);
            BW = BW | addedRegion;
            
            % Set background pixels where BW is false to zero.
            raw_img(repmat(~BW,[1 1 3])) = 0;
            hsv_img(repmat(~BW,[1 1 3])) = 0;
            
            [greenCenters, greendRadii] = self.blobImageCenter(raw_img, hsv_img, "green", 1); 
            % assign green return val
            gCent = greenCenters;

        end
        
        % Takes in a boolean of whether the robot has grabbed the knife yet
        % If the robot is knifeless, returns the coordinates of the center
        % of the knife. Otherwise returns the coordinates of the center of
        % the duck
        function Coords= stab(self, armed)
            % Get HSV image of current workspace
            raw_img = self.getImage(); 
            hsv_img = rgb2hsv(raw_img);     
            
            % Create empty mask of the correct size
            BW = false(size(raw_img,1),size(raw_img,2));
            
            % cutoff the image only to the checkerboard
            corners = self.findCheckerboardCorners();
            
            % get the image mask size
            m = size(BW, 1);
            n = size(BW, 2);
            
            % create a quad-polygon and mask the whole image
            addedRegion = poly2mask(corners(:, 1), corners(:, 2), m, n);
            BW = BW | addedRegion;
            
            % Set background pixels where BW is false to zero.
            raw_img(repmat(~BW,[1 1 3])) = 0;
            hsv_img(repmat(~BW,[1 1 3])) = 0;
            
            if (armed == 0) % no knife yet
                [KnifeCenter, KnifeRadii] = self.blobImageCenter(raw_img, hsv_img, "blue", 0); 
                Coords = KnifeCenter;
            else % knif has been acquired
                [DuckCenter, DuckRadii] = self.blobImageCenter(raw_img, hsv_img, "yellow", 0); 
                Coords = DuckCenter;
            end
        end
       
        % takes in the hsv_image of the cropped region of interest, color
        % from list of blue, yellow, red, green, gray, orange
        % takes in a boolean isCircle, where true tries to find circles,
        % and false just returns any blob center
        % outputs the balls centers [x, y] and radii
        function [centers, radii] = blobImageCenter(self, raw_img, hsv_img, color, isCircle)      
            I = hsv_img;            
            
            switch color
                case 'orange'
                % Orange
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.075;
                channel1Max = 0.120;            
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.365;
                channel2Max = 0.757;            
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.554;
                channel3Max = 0.936;
            
                case 'red'
                % Red
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.000;
                channel1Max = 0.070;                
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.381;
                channel2Max = 0.772;                
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.453;
                channel3Max = 0.855;
                
                case 'green'
                % Green
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.328;
                channel1Max = 0.482;            
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.182;
                channel2Max = 0.695;            
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.294;
                channel3Max = 0.876;
            
                case 'gray'
                % Gray
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.275;
                channel1Max = 0.500;            
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.077;
                channel2Max = 0.152;            
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.419;
                channel3Max = 0.681;
            
                case 'yellow'
                % Yellow
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.129;
                channel1Max = 0.202;   
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.271;
                channel2Max = 0.822;
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.521;
                channel3Max = 0.952;
            
                case 'blue'
                % Blue
                % Define thresholds for channel 1 based on histogram settings
                channel1Min = 0.564;
                channel1Max = 0.632;            
                % Define thresholds for channel 2 based on histogram settings
                channel2Min = 0.420;
                channel2Max = 0.965;            
                % Define thresholds for channel 3 based on histogram settings
                channel3Min = 0.353;
                channel3Max = 0.984;
            
                otherwise
                    warning("Wrong f'ing color!");
            end
            
            % Create mask based on chosen histogram thresholds
            sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
                (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
                (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
            BW = sliderBW;
            
            % Initialize output masked image based on input image.
            maskedRGBImage = raw_img;            
            % Set background pixels where BW is false to zero.
            maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
            
            % Create grayscale masked image
            bw_img = rgb2gray(maskedRGBImage); 
            % bw_img = histeq(bw_img);
            
            % apply the mask calculated in the main function
            %             figure;imshow(maskedBWImage);
            %             title("Checkerb BWoard Masked Image");
            
            %% APPLY FILTERS IN ORDER
            % Median filter, erosion, dilation, binarization
            maskedBWImage = medfilt2(bw_img);
            
            %             figure;imshow(maskedBWImage);
            %             title("Medfilt2 Image");
            
            se = offsetstrel('ball', 3, 3);
            maskedBWImage = imerode(maskedBWImage, se);
            
            %             figure;imshow(maskedBWImage);
            %             title("Imerode Image");
            
            maskedBWImage = imdilate(maskedBWImage, se);
            
            %             figure;imshow(maskedBWImage);
            %             title("Imdilate Image");
            
            maskedBWImage = imbinarize(maskedBWImage, 0.0001);    
        
            % blob vs circle matching
            radius_min = 16;
            radius_max = 42;
            if isCircle
                [centers, radii] = imfindcircles(maskedBWImage, [radius_min radius_max], "ObjectPolarity","bright");
            
            else
                % find the centers and radii of the blobs
                stats = regionprops("table",maskedBWImage,"Centroid","MajorAxisLength","MinorAxisLength");
                centers = stats.Centroid;
                radii = mean([stats.MajorAxisLength stats.MinorAxisLength],2) / 2;                         
            
                % just graph the output with the overlayed centers
            %             fig = figure; imshow(maskedBWImage);
            %             hold on
            % %             viscircles(centers,radii); 
            % %             plot(centers(:,1),centers(:,2),'r*');
            %             hold off
            %             title("Masked Image " + color);
            %             drawnow;
            %             pause(0.1);
            
                % radii minimum and maximum filter in the image               
                centers = centers(radii >= radius_min, :);
                radii = radii((radii >= radius_min));
            end
        end
        
         % Takes in the current x, y, z, alpha pose that th robot is going
         % for and returns the corrected pose that remvoes the xy offset
         % from the camera detection
         function correctedPose = remove_xy_offset(self, currentPose)
            r = 11.5; % ball radius in mm
            h = 175; % camera lens height in mm
            cam_x = 340; % camera distance from robot in x direction
            
            % Change whether we add or subtract from y based on whether y
            % is positive or negative
            if(currentPose(2) < 0)
                adjust = (r/h);
            else
                adjust = -(r/h);
            end
            
            % Adjust x and y and return
            correctedPose = [currentPose(1)+(r/h)/3*(cam_x-currentPose(1)) currentPose(2)+adjust*currentPose(2)-3 currentPose(3) currentPose(4)]; % pose with adjusted x, y
         end
    end
end