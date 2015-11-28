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
% hw5_team_13(1,'hw4_world_and_obstacles_convex.txt', 'hw4_start_goal.txt');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw5_team_13(serPort) 
    
    % read image from linksys camera
    img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
    
    % convert to hsv
    img_hsv = rgb2hsv(img_rgb);
    
    robocam = figure();
%     imshow(img_hsv(:,:,1));
%     colormap('hsv')
%     colorbar;
    
    % choose a color value to follow
    imshow(img_rgb);
    [x,y] = ginput(1);
    x = round(x);
    y = round(y);
    target_color = img_hsv(y,x,1); % Hue to follow
    
    % robot follow target
    while(1)
        % read image from linksys camera
        img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
        [x,y,area] = getTarget(img_rgb, target_color, 0.03);

        display([x,y,area]);
        
    end

end

%%% get centroid and area of colored target object in a new captured image
function [x,y,area] = getTarget(img_rgb, hue, range)

    % convert to hsv
    img_hsv = rgb2hsv(img_rgb);
    
    % threshold image based on hue
    img_thresh = img_hsv(:,:,1) > hue - range &...
        img_hsv(:,:,1) < hue + range;
    
    % dilate and erode to remove noise
    img_thresh = bwmorph(img_thresh, 'erode', 5);
    img_thresh = bwmorph(img_thresh, 'dilate', 8);
    img_thresh = bwmorph(img_thresh, 'erode', 3);
    
    % create labeled image to find largest object as target
    [labeled_img, n] = bwlabel(img_thresh);
    
    % get area and centroids of objects/blobs
    stats = regionprops(labeled_img, 'Area', 'Centroid');
    
    % find largest blob to use as target
    largest_i = 1;
    for i=1:size(stats,1)
        if (stats(i).Area > stats(largest_i).Area)
            largest_i = i;
        end
    end
    
    x = round(stats(largest_i).Centroid(1));
    y = round(stats(largest_i).Centroid(2));
    area = round(stats(largest_i).Area);
    
    imshow(img_thresh);

end

%% ROBORACE %%%%%%%%%%%

function roborace(serPort, path)
    display(path);
    
    
    
    
    start_coordinates = path(1,:);
    goal_coordinates = path(size(path,1),:);
    
    currentX = start_coordinates(1);
    currentY = start_coordinates(2);
    
    current_angle = 0;
    
    display(currentX);
    display(currentY);
    
    % reset distance and angle odometry
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    % travel from start to goal
    for i=2:(size(path,1))
        next_x = path(i, 1);
        next_y = path(i, 2);
        
        % navigate to each vertex
        
        delta_x = (next_x - currentX);
        delta_y = (next_y - currentY);
        
        theta = atan2(-delta_x, delta_y);
        
        display('----------------------> ');
        display(180*theta/pi);

        % turnAngle is in degrees
        turnAngle(serPort, 0.2, (theta - current_angle) * 180 / pi);
        current_angle = theta;
        
        distance = sqrt(delta_x^2 + delta_y^2);
        dist_traveled = 0;
        
        % go straight
        while (dist_traveled <= distance)
            SetFwdVelAngVelCreate(serPort, 0.5, 0); % move forward
            dist_traveled = dist_traveled + DistanceSensorRoomba(serPort);
            pause(0.1);
        end
        
        stopRobot(serPort);
        
        currentX = next_x;
        currentY = next_y;
    end

    % reached goal
    stopRobot(serPort);
end

function stopRobot(serPort)
        SetFwdVelAngVelCreate(serPort, 0, 0); % Stop the Robot
end
