%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 2
% Program works in both simulator and real-life.
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% main function 
function hw3_team_13(serPort, goalDistance)
    
    fig1 = figure('Name','robot path'); % draw robot path
    %close(fig1);
    
    
    axes('XLim', [-2 6], 'YLim', [-4 4]);
    drawInterval = 0.5; % draw every 0.5 seconds
    dStart = tic;

    hitPoints = [];
    %goalDistance = 4;
    
    
    goalX = 4;
    goalY = 0;
    
    % Read Bumpers, call ONCE at beginning, resets readings
    BumpsWheelDropsSensorsRoomba(serPort);
    WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    
    % Initialize variables keeping track of distance travelled by roomba                                
    currentX = 0;
    currentY = 0;
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
                while (~WallSensorReadRoomba(serPort))
                    turnAngle(serPort, turnVelocity, 2);
                    recordAngleTurn();

                    pause(0.1);    
                end
                turnAngle(serPort, turnVelocity, 66);
                recordAngleTurn();
        
                state = 1;  
            else
                
                dX = goalX - currentX;
                dY = goalY - currentY;
                distanceFromGoal = sqrt(dX * dX + dY + dY);
                if (distanceFromGoal < 0.05)
                    display('reached goal');
                    state = 2;
                end
                
                
            end
        end
        
        if (state == 1)
            followWall(1);
            
            pdX = goalX - previousX;
            pdY = goalY - previousY;
            previousDistance = sqrt(pdX * pdX + pdY * pdY);
            
            cdX = goalX - currentX;
            cdY = goalY - currentY;
            currentDistance = sqrt(cdX * cdX + cdY * cdY);
            
            
            ddX = currentX - previousX;
            ddY = currentY - previousY;
            deltaDistance = sqrt(ddX * ddX + ddY * ddY);
            
            
            disp([' previousX:',num2str(previousX),' previousY:',num2str(previousY),...
          ' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y),...
          ' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);

      
      
            if (deltaDistance < 0.1)
                display('back to where it started -- STUCK!');
                
                state = 2;
            elseif (currentDistance < previousDistance && previousDistance > deltaDistance)
                % we got closer!
                display('got closer!');
                
                
                reorientAngle = -(currentA - previousA);
                reorientDegrees = 180 * reorientAngle / pi;
                display(reorientDegrees);
                turnAngle(serPort, turnVelocity, reorientDegrees);
                recordAngleTurn();
                
                
                state = 0;
            else
                display('got farther!');
                state = 1;
            end

        end
        
        if (state == 2)
            
            stopRobot();
            
            return;
        end
            
            
            
            
            
            
         
        

        
        % dStart = mapRobot(dStart,drawInterval,fig1, currentX, currentY, currentA);

        recordRobotTravel(); % update distance traveled
        pause(0.1);

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
                turnAngle(serPort, turnVelocity, 5);
            elseif (BumpLeft)
                turnAngle(serPort, turnVelocity, -5);
            elseif (BumpFront)
                turnAngle(serPort, turnVelocity, 25);
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
            
            recordRobotTravel();
            
            delta_x = currentX - previousX; % return: total change in x from starting point
            delta_y = currentY - previousY;
        
            if (~outOfMargin && abs(delta_x) > 0.05 && abs(delta_y) > 0.05)
                outOfMargin = 1;
            end
            
            % check if back to origin, or on m-line
            if (outOfMargin) 
                display('---------------------------->OUT OF MARGIN');
                if (abs(delta_y) < 0.05 && m_line == 1)
                    %%% on the m-line %%%
                    stopRobot();
                    return;
                end
                
                if (abs(delta_x) < 0.05 && abs(delta_y) < 0.05)
                    %%%% back to the original spot %%%%
                    stopRobot();
                    return;
                end
                
            end
            
            pause(0.1);
        end
    end

    function recordAngleTurn()
        currentA = currentA + AngleSensorRoomba(serPort);
    end
    % record robot's distance traveled from last reading
    function recordRobotTravel() 
      recordAngleTurn();

      distance = DistanceSensorRoomba(serPort);

      currentX = currentX + distance * cos(currentA);
      currentY = currentY + distance * sin(currentA);

      disp(['currentA:',num2str(180 * currentA/pi),...
          ' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y),...
          ' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);

      %total_offset = power(delta_x, 2) + power(delta_y, 2);
      %display(['total_offset: ', num2str(total_offset)]);
    end

    
    function stopRobot()
        SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
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
