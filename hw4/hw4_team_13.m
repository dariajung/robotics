%  COMS W4733 Computational Aspects of Robotics 2015

function hw4_team_13(serPort, worldFile, startGoal) 

    robotDiameter = 0.35;

    [wall, obstacles] = readWorldFile(worldFile);
    
    % Plotting for visual verification
    plotObject(wall, 0, 0.8, 1);
    
    grownObstacles = cell(1, size(obstacles, 1));
        
    for i = 1:size(obstacles, 2)
        plotObject(obstacles{i}, 0, 0.8,1);
        % call Grow function here
        grownObstacles{1, i} = growObstacle(obstacles{i}, robotDiameter);
        
        plotObject(grownObstacles{1, i}, 1, 0, 0);
    end
    
    celldisp(grownObstacles);
end

% first integer gives you the number of obstacles
% for each obstacle:
%   first integer gives you the number of vertices
%   the vertices follow as X Y pairs, one per line, each with two coordinates
% you should close the obstacle by linking the last vertex to the first
% the first obstacle in the file actually specifies the wall that encloses the working environment.

function [wall, obstacles] = readWorldFile(file)

    try
        fid = fopen(file, 'r');
        line = fgets(fid);
               
        numObstacles = str2double(line);
        
        % First obstacle is wall
        wall = readObstacle(fid);
       
        obstacles = cell(1, numObstacles - 1);
        
        for i = 1:(numObstacles - 1)
            obstacles{1, i} = readObstacle(fid);
        end
        
        % close file
        fclose(fid);
        
    catch
        display('Failed to open world file')
        wall = zeros(0, 1);
        obstacles = {};
    end
end

% first integer gives you the number of vertices
% the vertices follow as X Y pairs, one per line, each with two coordinates
function obstacle = readObstacle(file)
    line = fgets(file);
    numVerticies = str2double(line);
    
    obstacle = zeros(numVerticies, 2);
    
    for i = 1:numVerticies 
        % read in a line from fid
        line = fgets(file);
        vertex = strsplit(line);
        
        obstacle(i,:) = [str2double(vertex(1)), str2double(vertex(2))];
    end
    
end

% Passing in a matrix to plot
function plotObject(object, r, g, b) 

    prev_x = object(1, 1);
    prev_y = object(1, 2);
    
    for i = 2:size(object, 1)
        x = object(i, 1);
        y = object(i, 2);
        
        line([prev_x, x], [prev_y, y], 'LineWidth', 1, 'Color', [r, g, b]);
        
        prev_x = x;
        prev_y = y;
    end

    line([prev_x, object(1, 1)], [prev_y, object(1,2)], 'LineWidth', 1, 'Color', [0, 0.8, 1]);

end

function tempMat = growObstacle(obstacle, robotDiameter)
    [r,c] = size(obstacle);
    tempMat = zeros(r*4,c);
    
    for i = 1:size(obstacle, 1)
        x = obstacle(i, 1);
        y = obstacle(i, 2);
        
        tempMat(4*(i - 1) + 1, 1) = x;
        tempMat(4*(i - 1) + 1, 2) = y;
        
        tempMat(4*(i - 1) + 2, 1) = x;
        tempMat(4*(i - 1) + 2, 2) = y + robotDiameter;
        
        tempMat(4*(i - 1) + 3, 1) = x + robotDiameter;
        tempMat(4*(i - 1) + 3, 2) = y + robotDiameter;
        
        tempMat(4*(i - 1) + 4, 1) = x + robotDiameter;
        tempMat(4*(i - 1) + 4, 2) = y;
    end   
end

