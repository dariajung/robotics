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

% hw5_team_13(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw5_colortracker_team_13(serPort)

    % reset distance and angle odometry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    robocam = figure();
    
    % read initial image from linksys camera
    img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
    
    % image center, to center target
    camera_x = size(img_rgb,2)/2;
    
    % convert to hsv
    img_hsv = rgb2hsv(img_rgb);
%     imshow(img_hsv(:,:,1));
%     colormap('hsv')
%     colorbar;
    
    % choose a color value to follow
    imshow(img_rgb);
    [x,y] = ginput(1);
    x = round(x);
    y = round(y);
    target_color = img_hsv(y,x,:); % Hue to follow
    
    [~,~,obj_area] = getTarget(img_rgb, target_color, 0.03, 0.5);
    
    robot_turn = 0;
    % robot follow target
    while(1)
        % read image from linksys camera
        img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
        [obj_x,~,area] = getTarget(img_rgb, target_color, 0.03, 0.5);
        
        display('camera_x, obj_x, obj_area, current_area ----->');
        display([camera_x, obj_x, obj_area, area]);
        
        
        % no object found in frame
        if (area < 0)
            % continue;
            SetFwdVelAngVelCreate(serPort, 0, robot_turn);
        end
        
        % error range of 25 pixels where the camera doesn't need to turn
        if (obj_x > camera_x + 25)
            display('turning right');
            robot_turn = -0.1;
            SetFwdVelAngVelCreate(serPort, 0, -0.1);
            pause(0.005);
            % turnAngle(serPort, 0.2, -1);
        elseif (obj_x < camera_x - 25)
            display('turning left');
            robot_turn = 0.1;
            SetFwdVelAngVelCreate(serPort, 0, 0.1);
            pause(0.005);
            % turnAngle(serPort, 0.2, 1);
        else 
            display('stopping roomba turn');

            if (area < obj_area - 290)
                % move forward
                SetFwdVelAngVelCreate(serPort, 0.05, 0);
            elseif (area > obj_area + 290)
                % move backward
                SetFwdVelAngVelCreate(serPort, -0.05, 0);
            else
                display('stop robot if out of frame');
                SetFwdVelAngVelCreate(serPort, 0, 0);
            end
            pause(0.005);
        
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