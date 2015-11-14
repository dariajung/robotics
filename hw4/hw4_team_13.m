%  COMS W4733 Computational Aspects of Robotics 2015

function hw4_team_13(serPort, worldFile, sgFile) 

    robotDiameter = 0.35;

    [wall, obstacles] = readWorldFile(worldFile);
    [start, goal] = readStartGoal(sgFile);
    
    display([start, goal]);
    
    % Plotting for visual verification
    plotObject(wall, 0, 0.8, 1);
    
    hold on;
    
    grownObstacles = cell(1, size(obstacles, 1));
    
    grown_vertices = 0;
        
    for i = 1:size(obstacles, 2)
        plotObject(obstacles{i}, 0, 0.8,1);
        % call Grow function here
        grownObstacles{1, i} = growObstacle(obstacles{i}, robotDiameter);
        grown_vertices = grown_vertices + size(grownObstacles{1,i},1);
        
        plotObject(grownObstacles{1, i}, 1, 0, 0);
    end
    
    display(grown_vertices);
    
    generateVisibilityGraph(start, goal, grownObstacles, wall, grown_vertices);
    
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
    
%     display(tempMat(:,1))
%     display(tempMat(:,2))
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
    
    for i = 2:size(obstacle, 1)
        x = obstacle(i, 1);
        y = obstacle(i, 2);
        
        % line([prev_x, x], [prev_y, y], 'LineWidth', 1, 'Color', [r, g, b]);
        
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

function [verticies, edges] = generateVisibilityGraph(start, goal, obstacles, wall, total_obst_verticies)
    % first, generate all possible edges between obstacles
    % second, generate all edges of the wall
    
    obstacles = [obstacles, {start, goal, wall}];
    
    obst_edges = [];
    
    hold on;
    
    for i = 1:size(obstacles, 2)
        temp = getObstacleEdges(obstacles{1, i});
        
        display(temp);
        
        obst_edges = vertcat(obst_edges, temp);
        plotEdge(temp, 0, 1, 0);
    end
    
%     wall_edges = getObstacleEdges(wall);
%     plotEdge(wall_edges, 0, 1, 0);
   
%     display(obst_edges)
%     display(size(obst_edges))

    hold on;

    % go through all obstacles
    for i = 1:size(obstacles, 2)
        obst = obstacles{1, i};
        
        % go through each vertex in an obstacle
        for j = 1:size(obst, 1)
            display(obst(j, 1));
            display(obst(j, 2));
            
            % go through next obstacle
            for k = (i+1):size(obstacles, 2)
                obst2 = obstacles{1, k};

                % go through each vertex in an obstacle
                for l = 1:size(obst2, 1)
                    display(obst2(l, 1));
                    display(obst2(l, 2));
                    
                    temp_edge = [obst(j, 1), obst(j, 2), obst2(l, 1), obst2(l, 2)]
                    
                    obst_edges = vertcat(obst_edges, temp_edge);
                end
            end
        end
    end
    
    display(size(obst_edges, 1)); 
    plotEdge(obst_edges, 0, 1, 0);
    
    mapObj = containers.Map;
    
    for i = 1:size(obst_edges, 1)
        for j = 1:size(obst_edges, 1)
            if (obst_edges(i,:) == obst_edges(j,:))
                continue;
            end
            edge1 = obst_edges(i,:);
            edge2 = obst_edges(j,:);
            
            if (isKey(mapObj, (mat2str(edge1))))
                mb1 = mapObj(mat2str(edge1));
                m1 = mb1(1);
                b1 = mb1(2);
                
            else
                [m1, b1] = getLine(edge1);
                mapObj(mat2str(edge1)) = [m1, b1];
            end
       
            
            if (isKey(mapObj, (mat2str(edge2))))
                mb2 = mapObj(mat2str(edge2));
                m2 = mb2(1);
                b2 = mb2(2);
            else
                [m2, b2] = getLine(edge2);
                mapObj(mat2str(edge2)) = [m2, b2];
            end
            
            [x, y] = computeIntersection(m1, b1, m2, b2);
            %display([x, y]);
            if (~isnan(x) && ~isnan(y))
                plot(x, y, 'Marker','o','MarkerFaceColor','red','MarkerSize',6);
            end
        end
    end
    
    
    verticies = [];
    edges = [];

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
    x1 = edge(1,1);
    y1 = edge(1,2);
    x2 = edge(1,3);
    y2 = edge(1,4);
    
    dx = x1 - x2;
    
    if (dx == 0)
        % vertical line
        m = inf;
        b = x1;
        
    else
        
        m = (y1 - y2) / dx;
        b = y1 - m*x1;
    
    end
    display(m);
end
