%  COMS W4733 Computational Aspects of Robotics 2015

% TODO: get rid of one edge is outside wall
function hw4_team_13(serPort, worldFile, sgFile) 

    robotDiameter = 0.35;

    [wall, obstacles] = readWorldFile(worldFile);
    [start, goal] = readStartGoal(sgFile);
    
    % Plotting for visual verification
    plotObject(wall, 0, 0.8, 1);
    
    hold on;
    
    grownObstacles = cell(1, size(obstacles, 1));
    
    num_vertices = 0;
        
    for i = 1:size(obstacles, 2)
%         plotObject(obstacles{i}, 0, 0.8,1);
        % call Grow function here
        grownObstacles{1, i} = growObstacle(obstacles{i}, robotDiameter);
        num_vertices = num_vertices + size(grownObstacles{1,i},1);       
%         plotObject(grownObstacles{1, i}, 1, 0, 0);
    end   
    % num_vertices = 37, wall = 16
   
    vgraph = generateVisibilityGraph(start, goal, grownObstacles, wall);
    
%     for i = 1:size(grownObstacles, 2)
%         plotObject(grownObstacles{1,i}, 0, 0.8,1);  
%     end
    %celldisp(grownObstacles);
	%display(grownObstacles{1, 1});
end

function [start, goal] = readStartGoal(file)
    try
        fid = fopen(file, 'r');
        line = fgets(fid);
        
        start_0 = strsplit(line);
        line = fgets(fid);
        goal_0 = strsplit(line);
        
        start = [str2double(start_0(1)),str2double(start_0(2))];
        goal = [str2double(goal_0(1)),str2double(goal_0(2))];
      
    catch
        display('Couldn''t open start_goal file')
        start = [0,0];
        goal = [0,0];
    end
end

%% READ IN FILES %%%%

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

%% PLOT OBJECT %%%%
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

    line([prev_x, object(1, 1)], [prev_y, object(1,2)], 'LineWidth', 1, 'Color', [r, g, b]);

end

function plotEdge(object, r, g, b) 

%     display(object)
% 
%    if size(object, 1) == 1
%        plot(object(1), object(2), '.r');
%        return;
%    end

   for i = 1:size(object,1)
       line([object(i,1), object(i,3)], [object(i, 2), object(i, 4)], 'LineWidth', 1, 'Color', [r, g, b]);
   end

end

%% GROW OBSTACLE %%%%%%%
function bigObstacle = growObstacle(obstacle, robotDiameter)
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
        
    k = convhull(tempMat(:,1), tempMat(:,2), 'simplify', true);
    
    bigObstacle = zeros(size(k, 1) - 1, 2);
    
    for j = 1:size(bigObstacle)
        bigObstacle(j,1) = tempMat(k(j),1);
        bigObstacle(j,2) = tempMat(k(j),2);
    end
end

%% VISIBILITY GRAPH %%%%%

function [obstacle_edges] = getObstacleEdges(obstacle)

    if size(obstacle, 1) == 1
        obstacle_edges = [obstacle(1, 1), obstacle(1, 2), obstacle(1, 1), obstacle(1, 2)];
        return;
    end
    
    % Each row is x1y1, x2y2 which forms an edge pair
    obstacle_edges = zeros(size(obstacle, 1), 4);

    prev_x = obstacle(1, 1);
    prev_y = obstacle(1, 2);
    x = 0; y = 0;
    for i = 2:size(obstacle, 1)
        x = obstacle(i, 1);
        y = obstacle(i, 2);

        % store edge
        obstacle_edges(i, 1) = prev_x;
        obstacle_edges(i, 2) = prev_y;
        obstacle_edges(i, 3) = x;
        obstacle_edges(i, 4) = y;

        prev_x = x;
        prev_y = y;
    end
    
    obstacle_edges(1, 1) = x;
    obstacle_edges(1, 2) = y;
    obstacle_edges(1, 3) = obstacle(1, 1);
    obstacle_edges(1, 4) = obstacle(1, 2);
end

% possible_paths, every row is 1x4 x1,y1,x2,y2 represents edge
function possible_paths = generateVisibilityGraph(start, goal, obstacles, wall)
    % generate just obstacle edges
    original_obstacles = obstacles;

    % first, generate all possible edges between obstacles
    % second, generate all edges of the wall
    
    obstacles_with_wall = [{wall}, obstacles];
    
    celldisp(obstacles_with_wall);
    
    possible_paths = [];
    
    hold on;

    
    % go through all obstacles (minus wall, start, goal)
    numObstacles = size(original_obstacles, 2);
    for i = -1:size(obstacles_with_wall, 2)
    %for i = -1:0    
        startgoal = true;
        if (i == -1)
            obst1 = start;
            display('start point');
            display(start);
        elseif (i == 0)
            obst1 = goal;
            display('goal point');
            display(goal);
        else
            startgoal = false;
            obst1 = obstacles_with_wall{1, i};
        end
        
        
        % for each vertex of obstacle
        for j = 1:size(obst1, 1)
            
            v1_x = obst1(j, 1);
            v1_y = obst1(j, 2);

            % check if vertex inside another obstacle
            
            % check if v1 is inside another obstacle
            isInside = false;
            for k = 1:numObstacles
                if (i ~= k)
                    in = insideObstacle(v1_x, v1_y, original_obstacles{1,k});
                    if (in)
                        isInside = true;
                        plot(v1_x, v1_y, '.r');
                        break;
                    end
                end
            end
            if (isInside == false)
                % point not inside an obstacle, try to make an edge
                
                if (i < 1)
                    startIndex = 1;
                else
                    startIndex = i + 1;
                end
                
                
                for k = startIndex:size(obstacles_with_wall,2)
                    obst2 = obstacles_with_wall{1,k};
                    for l = 1:size(obst2, 1)
                        % each vertex in obstacle 2
                        v2_x = obst2(l, 1);
                        v2_y = obst2(l, 2);
                        
                        isInside2 = false;
                        for n = 1:numObstacles
                            if (k ~= n)
                                in = insideObstacle(v2_x, v2_y, original_obstacles{1,n});
                                if (in)
                                    isInside2 = true;
                                    plot(v2_x, v2_y, '.r');
                                    
                                    break;
                                end
                            end
                        end
                        % can possibly make an edge! (both vertices are not
                        % inside an obstacle)
                        if (isInside2 == false)
                            
                            temp_edge = [v1_x, v1_y, v2_x, v2_y];
                            
                            %check if this edge intersects any obstacles
                            interObst = false;
                            for n = 1:numObstacles
                                testObst = original_obstacles{1,n};
                                in = intersectObstacle(temp_edge, testObst);
                                if (in)
                                    interObst = true;
                                    break;
                                end
                            end
                            
                            in = intersectObstacle(temp_edge, wall);
                            
                            if (interObst == false && in == false)
                                % edge is safe!
                                possible_paths = vertcat(possible_paths, temp_edge);
                            end
                        end
                    end
                end
            end
        end
    end
    
%     obst_edges = [];
    for i = 1:size(obstacles_with_wall, 2)
        temp = getObstacleEdges(obstacles_with_wall{1, i});
        possible_paths = vertcat(possible_paths, temp);
    end
%     display(size(obst_edges,1));
   
    
    for i = 1:size(possible_paths, 1)
        temp_edge = possible_paths(i,:);
        line([temp_edge(1), temp_edge(3)], [temp_edge(2), temp_edge(4)], 'LineWidth', 1, 'Color', [0, 0, 0]);
    end
end

function b = intersectObstacle(edge, obstacle)

    obstacle_edges = getObstacleEdges(obstacle);
    
    b = false;

    % goes through edges of obstacle
    for i=1:size(obstacle_edges)    
        currEdge = obstacle_edges(i,:);

        if (isequal(edge, currEdge)) 
            continue;
        end

        [p,b] = edgesIntersect(edge, currEdge);

        if (b == true)
            return;
        end
    end
end


function b = insideObstacle(x, y, obstacle)

    [in,on] = inpolygon(x,y,obstacle(:,1),obstacle(:,2));
    b = in && ~on;

end
function [p,b] = edgesIntersect(edge1, edge2)
    
    p1 = [edge1(1),edge1(2)];
    p2 = [edge1(3),edge1(4)];
    p3 = [edge2(1),edge2(2)];
    p4 = [edge2(3),edge2(4)];
    
    b = false;
    % edges form a corner, ok
    if (isequal(p1, p3) || isequal(p1, p4))
        p = p1;
        return;
    elseif (isequal(p2, p3) || isequal(p2, p4))
        p = p2;
        return;
    end
    
    [m1, b1] = getLine(edge1);
    [m2, b2] = getLine(edge2);
    
    [px, py] = computeIntersection(m1, b1, m2, b2);
    if (isnan(px))
        p = [inf, inf];
        return; 
    end
        
    b = true;
    d1 = pdist([p1;p2]);
    d2 = pdist([p3;p4]);
    
    myEps = 1e-12;
    
    p = [px, py];
    if (pdist([p;p1]) - d1 > myEps || pdist([p;p2]) - d1 > myEps || ...
            pdist([p;p3]) - d2 > myEps || pdist([p;p4]) - d2 > myEps)
        b = false;
    end
 
    
%     b = false;
%     if (px >= min([p3(1),p4(1)]) && px <= max([p3(1),p4(1)]))
%        if (px >= min([p1(1),p2(1)]) && px <= max([p1(1),p2(1)]))
%            
%            if (py >= min([p3(2),p4(2)]) && py <= max([p3(2),p4(2)]))
%                
%                if (py >= min([p1(2),p2(2)]) && py <= max([p1(2),p2(2)]))
%                    
%                    b = true;
%                    
%                end   
%            end
%        end
%     end
    
    
    
    

end

%% Point of Intersection %%%%%%%%%%%%
function [x, y] = computeIntersection(m1, b1, m2, b2)  
    if (m1 == m2) 
        x = NaN;
        y = NaN;
    elseif (m1 == inf)
        x = b1;
        y = m2 * x + b2;
    elseif (m2 == inf)
        x = b2;
        y = m1 * x + b1;
    else
        x = (b2 - b1) / (m1 - m2);
        y = m1*x + b1;
    end
end

%% Compute line %%%%%%%%%%%%

function [m, b] = getLine(edge)
    x1 = double(edge(1,1));
    y1 = double(edge(1,2));
    x2 = double(edge(1,3));
    y2 = double(edge(1,4));
    
    dx = x1 - x2;
    
    % if vertical, m = inf, b = x-intercept
    if (dx == 0)
        % vertical line
        m = inf;
        b = x1;
        
    else
        
        m = double((y1 - y2) / dx);
        b = y1 - m*x1;
    
    end
end
