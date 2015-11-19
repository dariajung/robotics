%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 4
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TODO: get rid of one edge is outside wall
% convex hull
% extra credit: dealing with bumping objects early
% test robot
% cleanup code

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
   
    for i = 1:size(obstacles, 2)
        plotObject(obstacles{i}, 0, 0.8,1);
    end
    %celldisp(grownObstacles);
	%display(grownObstacles{1, 1});
    
    
    % convert visgraph to adjacency matrix for dijkstra's
    [adjacency_matrix, verticies] = vis2adjmat(vgraph, start);
    
    % find shortest path from start to goal
    shortest_path = getShortestPath(start, goal, adjacency_matrix, verticies, robotDiameter);
    
    % roborace
    roborace(serPort, shortest_path);
    
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
        
%     k = convexHull(tempMat);
%     
%     bigObstacle = zeros(size(k, 1) - 1, 2);
%     
%     for j = 1:size(bigObstacle)
%         bigObstacle(j,1) = tempMat(k(j),1);
%         bigObstacle(j,2) = tempMat(k(j),2);
%     end
% %     display('bigobst');
% %     display(bigObstacle);

    bigObstacle = convexHull(tempMat);

end

function cvhull = convexHull(points)

    originalPoints = points;
    
    % points with minimum y
    minY = points(points(:,2) == min(points(:,2)),:);
    
    % minimum y and min x (bottom left most point)
    p0 = minY(minY(:,1) == min(minY(:,1)),:);
    display(p0);
    
    numP = size(points,1);
    points = horzcat(points, zeros(numP,2)); % add field for angle and distance
    for i=1:numP
       px = points(i,1);
       py = points(i,2);
       if (~isequal(p0,[px,py]))
          dx = px - p0(1);
          dy = py - p0(2);
          da = atan2(dy,dx);
          
          points(i,3) = 180 * da/pi;
          points(i,4) = sqrt(dx^2 + dy^2);
          
       end
        
    end
    
    % sort by distance
    points = sortrows(points,4);
    % sort by angle
    points = sortrows(points,3);
    display(points);
    
    points = points(:,1:2);
    
    display(points);
    
    % push p0,p1
    p = points(1,:);
    q = points(2,:);

    cvhull = p;
    cvhull = vertcat(cvhull, q);

    for i=3:numP
       r = points(i,:);
       
       % normalized vectors
%        v1 = [p2(1) - p1(1), p2(2) - p1(2)];
%        mv1 = sqrt(v1(1)^2 + v1(2)^2);
%        v1(1) = v1(1) / mv1;
%        v1(2) = v1(2) / mv1;
%        
%        v2 = [p3(1) - p2(1), p3(2) - p2(2)];
%        mv2 = sqrt(v2(1)^2 + v2(2)^2);
%        v2(1) = v2(1) / mv2;
%        v2(2) = v2(2) / mv2;
%        
%        va = atan2(v2(2), v2(1)) - atan2(v1(2),v1(1));
%        display([p1(1),p1(2),p2(1),p2(2),p3(1),p3(2),180 - 180 * va/pi]);
       
        while (orientation(p,q,r) ~= 2)

           
            cvhull = cvhull(1:size(cvhull,1)-1,:);
           
            if (size(cvhull,1) > 1)
                q = cvhull(size(cvhull,1),:);
                p = cvhull(size(cvhull,1)-1,:);
            else
                break;
            end
           
           display('while');
       end
       cvhull = vertcat(cvhull,r);
           
       p = q;
       q = r;
        
    end

%     display(points);
%     display(originalPoints);
%     cvhull2 = cvhull;
%     cvhull = convhull(originalPoints(:,1), originalPoints(:,2), 'simplify', true);
    
end

function o = orientation(p,q,r)

%     display([p,q,r]);
    val = (q(2) - p(2)) * (r(1) - q(1)) - (q(1) - p(1)) * (r(2) - q(2));
    
    %display(val);
    if (val == 0)
        o = 0;
    elseif(val > 0)
        o = 1;
    else
        o = 2;
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
    
%     celldisp(obstacles_with_wall);
    
    possible_paths = [];
    
    hold on;

    
    % go through all obstacles (minus wall, start, goal)
    numObstacles = size(original_obstacles, 2);
    for i = -1:size(obstacles_with_wall, 2)
        startgoal = true;
        if (i == -1)
            obst1 = start;
            %display('start point');
            %display(start);
        elseif (i == 0)
            obst1 = goal;
            %display('goal point');
            %display(goal);
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
    
    % adding in obstacle walls
    for i = 0:size(obstacles_with_wall, 2)
        
        if (i == 0)
            obst_edges = [start(1),start(2),goal(1),goal(2)];
        else
            obst_edges = getObstacleEdges(obstacles_with_wall{1, i});
        end
        
        for j=1:size(obst_edges, 1)
            temp_edge = obst_edges(j,:);
                            
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
   
    % draw paths
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

%% DIJKSTRA'S %%%%%%%%%%

% path = matrix of x, y coordinates for each step in path:
% [ start_x start_y ]
% [ first_x first_y ]
% ....
% [ goal_x goal_y ]
function path = getShortestPath(start, goal, adjmat, verticies, robotDiameter)
    num_verticies = size(adjmat,1);
   
    % initialize matrix of distances from start to each vertex
    distances = inf(num_verticies,1);
    distances(1) = 0;   % distance from source to itself is 0
    
    prev_vertices = cell(num_verticies,1);
    visited = zeros(num_verticies,1);
    unvisited = num_verticies;
    
    % calculate shortest distance to each vertex
    while unvisited > 0
        [min_vertex, min_dist] = getMinVertex(distances, visited);
        visited(min_vertex) = 1;            % mark vertex as visited
        unvisited = unvisited - 1;

        % update distances of adjacent verticies to min_vertex
        for j=1:num_verticies
            
            % only update if not already visited
            if ~visited(j)
                temp_dist = min_dist + adjmat(min_vertex,j);
                
                % only update if distance from source to j smaller
                if temp_dist < distances(j)
                    distances(j) = distances(min_vertex) + adjmat(min_vertex,j);
                    prev_vertices{j} = min_vertex;
                end
            end         
        end
        
    end
        
    % get path from start to goal
    goal_idx = getGoalIndex(verticies, goal);    
    
    % initialize path to goal vertex
    path = goal;
    
    % backtrack until reach start vertex
    curr_idx = goal_idx;
    while curr_idx ~= 1
        if curr_idx ~= goal_idx
            vertex = verticies(curr_idx,:);
            path = vertcat(vertex, path);
        end
        curr_idx = prev_vertices{curr_idx};
    end
    
    % finished tracing back to start (curr_idx is 1)
    path = vertcat(start, path);
    
    display(path);
    
    % shift points to the left and down to match grown obstacle algorithm
    for i=1:size(path,1)

        path(i,1) = path(i,1) - robotDiameter/2;
        path(i,2) = path(i,2) - robotDiameter/2;
        
    end
    
    % plot shortest path
    for i=1:size(path,1)-1
        display(path(i,1));
        line([path(i,1), path(i+1,1)], [path(i,2), path(i+1,2)], 'LineWidth', 1, 'Color', [1, 0, 0]);
        
    end

    
end

function id = getGoalIndex(verticies, goal)
    id = 0;
    for i=1:size(verticies,1)
        if verticies(i,:) == goal
            id = i;
        end
    end
end

function [min_vertex, min_dist] = getMinVertex(distances, visited)
    min_dist = inf;
    
    for i=1:size(visited,1)
        if ~visited(i) && (distances(i) <= min_dist)
            min_dist = distances(i);
            min_vertex = i;
        end
    end

end

% create adjacency matrix from visibility graph
function [adjmat, verticies] = vis2adjmat(visgraph, start)
    % get unique verticies from visgraph
    [verticies, num_verticies] = getVerticies(visgraph, start);

    % create adjacency matrix
    adjmat = inf(num_verticies, num_verticies);
    
    for i=1:num_verticies
        for j=1:num_verticies
            vertex1 = verticies(i,:);
            vertex2 = verticies(j,:);
            
            % same vertex so distance is 0
            if vertex1 == vertex2
                adjmat(i,j) = 0;
            end
            
            % if edge between vertex1 and vertex2 exists in visgraph, get distance
            if ismember([vertex1 vertex2], visgraph, 'rows') || ismember([vertex2 vertex1], visgraph, 'rows')
%                 display('in visgraph');
%                 display([vertex1 vertex2]);
                
                adjmat(i,j) = pdist([vertex1;vertex2]);
            end
                
        end
    end         
end

function [vertex_list, num_verticies] = getVerticies(visgraph, start)
    % initialize vertex_list to first vertex in visgraph
    vertex_list = visgraph(1,1:2);

    % add verticies in visgraph to array of verticies
    for i=1:size(visgraph,1)
        vertex1 = visgraph(i,1:2);
        vertex2 = visgraph(i,3:4);
        
        if ~ismember(vertex1, vertex_list, 'rows')
            vertex_list = vertcat(vertex_list, vertex1);
        end
        
        if ~ismember(vertex2, vertex_list, 'rows')
            vertex_list = vertcat(vertex_list, vertex2);
        end
    end
    
    % check that start point is first vertex in vertex_list
    % if not, swap with first vertex
    if ~vertex_list(1,:) == start
        for i=1:size(vertex_list, 1)
            if vertex_list(i,:) == start
                temp = vertex_list(1,:);
                vertex_list(1,:) = start;
                vertex_list(i,:) = temp;
            end
        end     
    end
    
    num_verticies = size(vertex_list, 1);
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
        
        % reset to zero
        dist_traveled = 0;
        
        currentX = next_x;
        currentY = next_y;
    end

    % reached goal
    stopRobot(serPort);
end




% record robot's distance traveled from last reading
function recordRobotTravel(gridColor, robotOffset) 
%       if (toc(timeStart) > 60)
%           quit = true;
%       end  
        
      recordAngleTurn();

      distance = DistanceSensorRoomba(serPort);

%       currentX = currentX + distance * cos(currentA);
%       currentY = currentY + distance * sin(currentA);
% 
%       updateGrid(currentX,currentY,gridColor, robotOffset, false);
end

function recordAngleTurn()
        currentA = currentA + AngleSensorRoomba(serPort);
end

function stopRobot(serPort)
        SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
end
