%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HOW TO call function with obstacle files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% hw5_team_13(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw5_team_13_b(serPort)

    % reset distance and angle odometry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    % read image from linksys camera
    img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
    img_size = size(img_rgb, 1) * size(img_rgb, 2);
        
    obj_x = 1;
    obj_y = 1;
    camera_x = size(img_rgb,2)/2;
    
    robocam = figure();
    
    state = 0; % finding door color
    
    img_hsv = rgb2hsv(img_rgb);
    
    imshow(img_rgb);
    [x,y] = ginput(1);
    x = round(x);
    y = round(y);
    
    doorColor = img_hsv(y, x, :);
    
    display(doorColor);
    
%     [obj_x, obj_y, area] = getTarget(img_rgb, doorColor, 0.05, 0.5);
%     
%     display([obj_x, obj_y, area]);
    % moveTowardDoor(area, serPort);
    % moveRobotToDoorEdge(serPort, img_rgb, target_color)
    direction = -1;
    
    prev_area = 0;
    
    while(1)
        
        
        % Looking for blue of a door in CEPSR
        if (state == 0)
            img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
            
            [found_door, direction] = moveRobotToDoorEdge(serPort, img_rgb, doorColor);
            
            if (found_door == true)
                SetFwdVelAngVelCreate(serPort, 0, 0);
                state = 1;
                continue;
            else
                % make it go straight
                SetFwdVelAngVelCreate(serPort, .1, 0);
            end
            
            %state = 1;
            %continue;
        end
        
        % Moving towards door
        if (state == 1)
            [obj_x, obj_y, area] = getTarget(img_rgb, doorColor, 0.05, 0.5);
            moveTowardDoor(area, serPort);
            if (direction == 0)
                turnAngle(serPort, 0.2, 90);
            else
                turnAngle(serPort, 0.2, -90);
            end
            state = 2;
        end
        
        if (state == 2)
            display('state 2 -------------------');
            [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            bumped = BumpRight || BumpLeft || BumpFront;
            
            % if any bump sensor is 1, then the robot has reached door
            % go to state two
            if (bumped)
                state = 3;
                SetFwdVelAngVelCreate(serPort, 0, 0);
                continue;
            end
            
            SetFwdVelAngVelCreate(serPort, 0.3, 0);
        end
        
        % knocking state
        if (state == 3) 
            travelDist(serPort, 0.5, -0.05);
            travelDist(serPort, 0.5, 0.1);
            BeepRoomba(serPort);
            
            img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
            
            [~, ~, prev_area] = getTarget(img_rgb, doorColor, 0.05, 0.5);

            pause(1);
            state = 4;
        end
        
        if (state == 4)
            img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
            
            [o_x, ~, cur_area] = getTarget(img_rgb, doorColor, 0.05, 0.5);
            
            % check where center of door is, and compare it to camera center? o_x < camera_x
            if (prev_area > cur_area + 900)
                travelDist(serPort, 0.3, 1);
                state = 5;
            end
            
        end
        
        if (state == 5)
            break;
        end
        
%        % Moving towards door
%         if (state == 1)
%             
%             [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
%             bumped = BumpRight || BumpLeft || BumpFront;
%             
%             % if any bump sensor is 1, then the robot has reached door
%             % go to state two
%             if (bumped)
%                 state = 2;
%                 SetFwdVelAngVelCreate(serPort, 0, 0);
%                 continue;
%             end
%             
%              % read image from linksys camera
%             img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
%             [obj_x,obj_y,area] = getTarget(img_rgb, target_color, 0.03, 0.5);
% 
%             display('camera_x, obj_x ----->');
%             display([camera_x, obj_x]);
% 
%             % no object found in frame
%             if (area < 0)
%                 continue;
%             end
% 
%             % error range of 20 pixels where the camera doesn't need to turn
%             if (obj_x > camera_x + 25)
%                 display('turning right');
%                 SetFwdVelAngVelCreate(serPort, 0, -0.2);
%                 pause(0.005);
%                 % turnAngle(serPort, 0.2, -1);
%             elseif (obj_x < camera_x - 25)
%                 display('turning left');
%                 SetFwdVelAngVelCreate(serPort, 0, 0.2);
%                 pause(0.005);
%                 % turnAngle(serPort, 0.2, 1);
%             else 
%                 display('stopping roomba turn');
% 
%                 if (area < img_size - 200)
%                     % move forward
%                     SetFwdVelAngVelCreate(serPort, 0.05, 0);
%                     pause(0.005);
%                 else
%                     SetFwdVelAngVelCreate(serPort, 0, 0);
%                 end
% 
%             end
%         end
%         
%         % orienting towards door
%         if (state == 2)
%             % move until front-bump sensor 
%             [BumpRight, BumpLeft, ~, ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
%             
%             display('BUMP --------------------->');
%             display([BumpRight, BumpLeft, BumpFront]);
%             
%             %
%             if (BumpFront == 1) 
%                 state = 3;
%                 % SetFwdVelAngVelCreate(serPort, 0, 0);
%                 continue;
%             elseif (BumpRight == 1) 
%                 turnAngle(serPort, 0.1, -5);
%             elseif (BumpLeft == 1)
%                 turnAngle(serPort, 0.1, 5);
%             end
% 
%         end
%         
%         if (state == 3)
%             display('yay made it to state 3! ------------------------');
%         end

       
    end

end

function moveTowardDoor(area, serPort)
    m = -0.0006;
    b = 12.46;
    
    % distance is number of tiles
    num_tiles = area * m + b;
    
    tile_m = 0.305;
    
    % each tile is about 0.305 meters
    
    travelDist(serPort, 0.2, tile_m * num_tiles);
    
end

% move until right or left edge is the target blue
function [found_door, direction] = moveRobotToDoorEdge(serPort, img_rgb, target_color)
    found_door = false;
    direction = -1;

    % left or right based on if 
    num_cols = size(img_rgb, 2); % 320
    num_rows = size(img_rgb, 1); % 240
    display(num_cols)
    display(num_rows)
    
    right = img_rgb(:,num_cols - 20:num_cols, :);
    left = img_rgb(:,1:20, :);
    
    [obj_r_x, obj_r_y, r_area] = getTarget(right, target_color, 0.05, 0.5);
    [obj_l_x, obj_l_y, l_area] = getTarget(left, target_color, 0.05, 0.5);
    
    display([obj_r_x, obj_r_y, r_area]);
    display([obj_l_x, obj_l_y, l_area]);
    
    imshowpair(left, right, 'montage');
    
    rect_area = 20 * num_rows;
    display(rect_area);
    
    display(target_color);
    
    if (l_area > -1 || r_area > -1)
        found_door = true;
        if (l_area > -1) 
            direction = 0;
        else
            direction = 1;
        end
    end
    
end

%%% get centroid and area of colored target object in a new captured image
function [x,y,area] = getTarget(img_rgb, target_color, rangeH, rangeS)

    % convert to hsv
    img_hsv = rgb2hsv(img_rgb);
    
    % threshold image based on hue
    img_thresh_H = img_hsv(:,:,1) > target_color(1,1) - rangeH &...
        img_hsv(:,:,1) < target_color(1) + rangeH;
    
    % threshold image based on saturation
    img_thresh_S = img_hsv(:,:,2) > target_color(1,2) - rangeS &...
        img_hsv(:,:,2) < target_color(2) + rangeS;
    
    img_thresh = img_thresh_H & img_thresh_S;
    
    % dilate and erode to remove noise
    img_thresh = bwmorph(img_thresh, 'erode', 5);
    img_thresh = bwmorph(img_thresh, 'dilate', 8);
    img_thresh = bwmorph(img_thresh, 'erode', 3);
    
    imshowpair(img_rgb, img_thresh, 'montage');
    
    % create labeled image to find largest object as target
    [labeled_img, n] = bwlabel(img_thresh);
    
    display('number of objects:::::');
    display(n);
    if (n < 1)
        x = -1;
        y = -1;
        area = -1;
        return;
    end
    
    
    
    % get area and centroids of objects/blobs
    stats = regionprops(labeled_img, 'Area', 'Centroid');
    
    % find largest blob to use as target
    largest_i = 1;
    for i=1:size(stats,1)
        if (stats(i).Area > stats(largest_i).Area)
            largest_i = i;
        end
    end
    
    display(size(stats,1));
    display(largest_i);
    
    x = floor(stats(largest_i).Centroid(1)) + 1;
    y = floor(stats(largest_i).Centroid(2)) + 1;
    area = round(stats(largest_i).Area);
    
    display('-----------------');
    display([x, y, area]);
    display('-----------------');
    
    % draw a square at target center
    redTrackerY1 = y - 10;
    if (redTrackerY1 < 1)
        redTrackerY1 = 1;
    end
    redTrackerY2 = y + 10;
    if (redTrackerY2 > size(img_rgb, 1))
        redTrackerY2 = size(img_rgb, 1);
    end
    
    redTrackerX1 = x - 10;
    if (redTrackerX1 < 1)
        redTrackerX1 = 1;
    end
    redTrackerX2 = x + 10;
    if (redTrackerX2 > size(img_rgb, 2))
        redTrackerX2 = size(img_rgb, 2);
    end
    
    img_rgb(redTrackerY1:redTrackerY2,redTrackerX1:redTrackerX2,1) = 255;
  
    imshowpair(img_rgb, img_thresh, 'montage');

end


