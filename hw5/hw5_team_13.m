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

function hw5_team_13(serPort)

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
    target_color = img_hsv(y,x,1); % Hue to follow
    
    % robot follow target
    while(1)
        % read image from linksys camera
        img_rgb = im2double(imread('http://192.168.0.101/img/snapshot.cgi?'));
        [obj_x,obj_y,area] = getTarget(img_rgb, target_color, 0.03);
        
        display('camera_x, obj_x ----->');
        display([camera_x, obj_x]);
        
        if (obj_x > camera_x)
            display('turning right');
            turnAngle(serPort, 0.2, -1);
        elseif (obj_x < camera_x)
            display('turning left');
            turnAngle(serPort, 0.2, 1);
        end
        
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
    
    x = floor(stats(largest_i).Centroid(1)) + 1;
    y = floor(stats(largest_i).Centroid(2)) + 1;
    area = round(stats(largest_i).Area);
    
    display('-----------------');
    display([x, y, area]);
    display('-----------------');
    
    img_rgb(y-10:y+10,x-10:x+10,1) = 255;
    imshow(img_rgb);

end

%% ROBORACE %%%%%%%%%%%
function roborace(serPort, path)
    
    % go straight
    while (dist_traveled <= distance)
        SetFwdVelAngVelCreate(serPort, 0.5, 0); % move forward
        dist_traveled = dist_traveled + DistanceSensorRoomba(serPort);
        pause(0.1);
    end
    
end
function stopRobot(serPort)
        SetFwdVelAngVelCreate(serPort, 0, 0); % Stop the Robot
end
