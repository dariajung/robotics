%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 3
% Program works in both simulator and real-life.
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% TO DO

% - stopping after certain time
% - if red, turn, figure out if logic ok
% - take into account robot width
% - random turn into its own function
% - spiral in the beginning



% main function 
function hw3_group13(serPort, goalDistance)

    simulator_p = properties(serPort);
    
    rightTurn = 0;
    leftTurn = 0;
    frontTurn = 0;

    % simulator %
    if size(simulator_p,1) == 0
        rightTurn = 5;
        leftTurn = -5;
        frontTurn =25;
    else
        rightTurn = 20;
        leftTurn = -20;
        frontTurn = 40;
    end
    
    % INIT OCCUPANCY GRID
    % -1 (grey) = not visited
    % 0 (green) = visited, no obstacle
    % 1 (red) = visited, obstacle
    GREY = -1;
    GREEN = 0;
    RED = 1;
    colorState = 0;
    
    occFig = figure;
    
    gridSize = double(13.5/36);
    robotRadius = (13.5/2)/36;
    gridRange = 20;
    
    xRange = [-gridRange gridRange];
    yRange = [-gridRange gridRange];
    xlim(xRange);
    ylim(yRange);
    
    occGrid = zeros(2*gridRange, 2*gridRange);
    occGrid(:,:) = -1; % all grey
    
    cmap = [0.9,0.9,0.9]; % 1st row = red; 2nd = gray; 3rd = blue
    colormap(cmap);
    
    imagesc(xRange,yRange,occGrid);
    
    drawnow; % update grid
    grid on;
    hold on;
    
    quit = false;
    
    % random angles to choose from
    randomAngle = 20:10:80;

    %fig1 = figure('Name','robot path'); % draw robot path
    %close(fig1);
    
    
%     axes('XLim', [-2 6], 'YLim', [-4 4]);

    
    
    goalX = 4;
    goalY = 0;
    
    % Read Bumpers, call ONCE at beginning, resets readings
    BumpsWheelDropsSensorsRoomba(serPort);
    WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    
    % Initialize variables keeping track of distance travelled by roomba                                
    currentX = 0;
    currentY = 0;   
    
    timeStart = tic;
    
    currentA = 0;
    previousA = 0;
    previousX = 0;
    previousY = 0;
                
    total_offset = 0;
    
    delta_x = 0;
    delta_y = 0;
    
    % margin of error for sensor reading for stop position
    margin_error = power(0.25, 2);

    fwdVelocity = 0.08;
    turnVelocity = 0.1;
    
    prevGrid = [0, 0];
   

    % robot hit wall, reset sensors to start measuring change
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    % STARTING ROBOT
    state = 0; % going straight
                % state = 1; following wall
                
    while 1
        
        if (state == 0)
            SetFwdVelAngVelCreate(serPort, fwdVelocity, 0); % Move Forward
            
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            if (BumpRight || BumpLeft || BumpFront)
            
                stopRobot();

                previousA = currentA;
                previousX = currentX;
                previousY = currentY;

                % orient to wall
                travelDist(serPort, 0.05, -0.015); % move back from wall a bit initially
                if size(simulator_p,1) == 0
                    % SIMULATOR
                    while (~WallSensorReadRoomba(serPort))
                        turnAngle(serPort, turnVelocity, 2);
                        recordAngleTurn();

                        if size(simulator_p,1) == 0
                            % running in the simulator
                            pause(0.1);
                        end
                    end
                    turnAngle(serPort, turnVelocity, 66);
                else % real-life
                    turnAngle(serPort, turnVelocity, 90);
                end
                recordAngleTurn();
                
                
                if (checkGrid(currentX, currentY, 'front') ~= RED)
                    state = 1;
                    % updateGrid(currentX, currentY, RED, 'front', true)
                else
                    
                    newDirection = randomAngle(randi([1 size(randomAngle,2)],1));
                    if (BumpLeft)
                        newDirection = -1 * newDirection;
                    elseif (BumpFront)
                        newDirection = newDirection + 90;
                    end
                    
                    display('alreadyHitObstacle: turning to new direction!------------------------>');
                    display(newDirection);
                
                    turnAngle(serPort, turnVelocity, newDirection);
                    recordAngleTurn();
            
                end        
                
%             else
%                 
%                 dX = goalX - currentX;
%                 dY = goalY - currentY;
%                 distanceFromGoal = sqrt(dX * dX + dY + dY);
%                 if (distanceFromGoal < 0.05)
%                     display('reached goal');
%                     state = 2;
%                 end
%                 
%                 
            end
        end
        
        if (state == 1)
            followWall(0);
            if (quit == true) 
                display('=======================> EXITING PROGRAM <========================')
                return;
            end
            
%             pdX = goalX - previousX;
%             pdY = goalY - previousY;
%             previousDistance = sqrt(pdX * pdX + pdY * pdY);
%             
%             cdX = goalX - currentX;
%             cdY = goalY - currentY;
%             currentDistance = sqrt(cdX * cdX + cdY * cdY);
%             
%             
%             ddX = currentX - previousX;
%             ddY = currentY - previousY;
%             deltaDistance = sqrt(ddX * ddX + ddY * ddY);
            
            
%             disp([' previousX:',num2str(previousX),' previousY:',num2str(previousY),...
%           ' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y),...
%           ' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);

      
      
%             if (deltaDistance < 0.1)
                %display('back to where it started -- STUCK!'); % state = 2
                
            newDirection = randomAngle(randi([1 size(randomAngle,2)],1));
            display('followWall: turning to new direction!------------------------>');
            display(newDirection);
            turnAngle(serPort, turnVelocity, newDirection);
            recordAngleTurn();

            state = 0;
                
%             elseif (currentDistance < previousDistance && previousDistance > deltaDistance)
%                 % we got closer!
%                 display('got closer!');
%                 
%                 
%                 reorientAngle = -(currentA - previousA);
%                 reorientDegrees = 180 * reorientAngle / pi;
%                 display(reorientDegrees);
%                 turnAngle(serPort, turnVelocity, reorientDegrees);
%                 recordAngleTurn();
%                 
%                 
%                 state = 0;
%             else
%                 display('got farther!');
%                 state = 1;
%             end

        end
        
        if (state == 2)
            
            stopRobot();
            
            return;
        end
            
            
            
            
            
            
         
        

        
        % dStart = mapRobot(dStart,drawInterval,fig1, currentX, currentY, currentA);

        recordRobotTravel(GREEN, 'front'); % update distance traveled
        if size(simulator_p,1) == 0
            % running in the simulator
            pause(0.1);
        end
        
        if (quit == true)
            display('=======================> EXITING PROGRAM <========================')
            return;
        end


    end
    
    
    
    
    
    
    
    % 0 means no goal, return at start, else on m line
    function followWall(m_line)

        dStart = tic;
        
        
        total_offset = 0;
        originalWallAngle = 0;
        

        SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);

        outOfMargin = 0;
        while 1
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            
            if (BumpRight)
                turnAngle(serPort, turnVelocity, rightTurn);
            elseif (BumpLeft)
                turnAngle(serPort, turnVelocity, leftTurn);
            elseif (BumpFront)
                turnAngle(serPort, turnVelocity, frontTurn);
            end
            
            
            Bumps = BumpRight || BumpLeft || BumpFront;
            
            if (~WallSensorReadRoomba(serPort) && ~Bumps)

                stopRobot();
                SetFwdVelRadiusRoomba(serPort, fwdVelocity, -0.2);
            elseif (Bumps)
                stopRobot();
            else

                SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
            end
            
            recordRobotTravel(RED, 'side');
            
            delta_x = currentX - previousX; % return: total change in x from starting point
            delta_y = currentY - previousY;
        
            if (~outOfMargin && abs(delta_x) > 0.05 && abs(delta_y) > 0.05)
                outOfMargin = 1;
            end
            
            % check if back to origin, or on m-line
            if (outOfMargin) 
%                 display('---------------------------->OUT OF MARGIN');
                if (abs(delta_y) < 0.05 && m_line == 1)
                    %%% on the m-line %%%
                    stopRobot();
                    return;
                end
                
                if (abs(delta_x) < 0.08 && abs(delta_y) < 0.08)
                    %%%% back to the original spot %%%%
                    stopRobot();
                    return;
                end
                
            end
            if size(simulator_p,1) == 0
                % running in the simulator
                pause(0.1);
            end
            [c,r] = calculateGrid(currentX,currentY,'side');
            display('previous grid...................................');
            display(prevGrid);
            display([c,r]);
            if (checkGrid(currentX, currentY, 'side') == RED && prevGrid(1) ~= c && prevGrid(2) ~= r)
                break;
            end
            
            if (quit == true)
                return;
            end
        end
    end

    function recordAngleTurn()
        currentA = currentA + AngleSensorRoomba(serPort);
    end
    % record robot's distance traveled from last reading
    function recordRobotTravel(gridColor, robotOffset) 
      if (toc(timeStart) > 60)
          quit = true;
      end  
        
      recordAngleTurn();

      distance = DistanceSensorRoomba(serPort);

      currentX = currentX + distance * cos(currentA);
      currentY = currentY + distance * sin(currentA);
      

%       disp(['currentA:',num2str(180 * currentA/pi),...
%           ' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y),...
%           ' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);

      %total_offset = power(delta_x, 2) + power(delta_y, 2);
      %display(['total_offset: ', num2str(total_offset)]);
      
      
      % if following wall, set side grid to red and current grid to green
      if (strcmp(robotOffset, 'side'))
         % updateGrid(currentX,currentY,GREEN, 'center', false);
      end
      updateGrid(currentX,currentY,gridColor, robotOffset, false);
    end

    
    function stopRobot()
        SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
    end

    function gridVal = checkGrid(c,r, robotOffset)       
        [c,r] = calculateGrid(c,r,robotOffset);
        gridVal = occGrid(r,c);
    end

    function [x, y] = calculateGrid(c,r,robotOffset)
        if (strcmp(robotOffset, 'front'))
            c = c + robotRadius*cos(currentA);
            r = r + robotRadius*sin(currentA);
            
        elseif (strcmp(robotOffset,'side'))
            c = c + robotRadius*cos(currentA - pi/2);
            r = r + robotRadius*sin(currentA - pi/2);
        end
        
        c = round(c/ gridSize);
        r = round(r/ gridSize);
        
        c = c + gridRange;
        r = gridRange - r;
        
        x = c;
        y = r;
    end

    function updateGrid(c, r, value, robotOffset, force)
%         display(value);

        figure(occFig);
        hold on;
        
        if (strcmp(robotOffset, 'front'))
            c = c + robotRadius*cos(currentA);
            r = r + robotRadius*sin(currentA);
            
        elseif (strcmp(robotOffset,'side'))
            c = c + robotRadius*cos(currentA - pi/2);
            r = r + robotRadius*sin(currentA - pi/2);
        end
        
        c = round(c/ gridSize);
        r = round(r/ gridSize);
      
        
        c = c + gridRange;
        r = gridRange - r;
    
        if (occGrid(r,c) == -1 || force == true)
            occGrid(r,c) = value;
            prevGrid(1) = c;
            prevGrid(2) = r;
            timeStart = tic;
        end
        
        if (value == GREEN && colorState == 0)
            colorState = 1;
            cmap = [0.9,0.9,0.9;...
                    0.5,1,0.5]; % 1st row = red; 2nd = gray; 3rd = blue
            colormap(cmap);
        elseif (colorState == 1)
            colorState = 2;
            cmap = [0.9,0.9,0.9;...
                    0.5,1,0.5;...
                    1,0.5,0.5]; % 1st row = red; 2nd = gray; 3rd = blue
            colormap(cmap);
        end
        
       
        imagesc(xRange,yRange,occGrid);
        drawnow; % update grid
        plot(c, r, '.b');
        drawnow;
               
    end
end

% TODO:
% 
%     if reaches 0 again its inside a closed obstacle, exit report stuck
%         
% robot simulator keeps curving up, how to reorient back down if no
% obstacles?
% if goal is inside obstacle?
% instead of if theta > 0 theta or small number
