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
    goalDistance = 4;
    
    % Read Bumpers, call ONCE at beginning, resets readings
    BumpsWheelDropsSensorsRoomba(serPort);
    WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    
    % Initialize variables keeping track of distance travelled by roomba
    total_distance = 0;                                 
    currentX = 0;
    currentY = 0;
    currentA = 0;
    total_offset = 0;
    
    % margin of error for sensor reading for stop position
    margin_error = power(0.25, 2);

    wallVelocity = 0.08;
    turnVelocity = 0.1;
    
    function recordAngleTurn(serPort)
        currentA = currentA + AngleSensorRoomba(serPort);
    end
    % record robot's distance traveled from last reading
    function recordRobotTravel(serPort) 
        recordAngleTurn(serPort);
        
        distance = DistanceSensorRoomba(serPort);
        total_distance = total_distance + distance * 1.02; % * .97
        currentX = currentX + distance * cos(currentA);
        currentY = currentY + distance * sin(currentA);

        disp(['currentA:',num2str(180 * currentA/pi),...
            ' total_distance:',num2str(total_distance),...
            ' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);
        
        %total_offset = power(currentX, 2) + power(currentY, 2);
        %display(['total_offset: ', num2str(total_offset)]);
    end

    function stopRobot()
        SetFwdVelAngVelCreate(serPort, 0, 0); % Stop the Robot
    end

    % robot hit wall, reset sensors to start measuring change
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    % STARTING ROBOT
    SetFwdVelAngVelCreate(serPort, wallVelocity, 0); % Move Forward

    while 1
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        display('While loop currentX')
        display(currentX)
        
        recordRobotTravel(serPort); % update distance traveled
        display('------------------------------------->currentX GOING STRAIGHT');
        display(currentX)

        
        if (BumpRight || BumpLeft || BumpFront)
            
            stopRobot();
            
            hitPoints = [hitPoints; currentX, currentA]; % track angle also for thin walls
            [delta_x, currentA] = followWall(serPort, BumpRight, BumpLeft, BumpFront,...
                currentX, currentY, currentA, goalDistance, fig1, drawInterval);
            
            return
            
            
            
            
            
            
%           display(['DELTA_X::::::::::::: ', num2str(delta_x)])
%           display(['CURRENTANGLE:::::::: ', num2str(currentA)])
%          
%           currentX = currentX + delta_x;
% 
%           if (currentX > goalDistance)
%                 display('-------------------------------->THINKS ITS OVERSHOOTING')
%               turnAngle(serPort, turnVelocity, 180 - (currentA/pi)*180);
%               %display('------------------------------------->finished turnAngle after wall follow');
%               %AngleSensorRoomba(serPort); % reset angle sensor to 0
%             else
%               turnAngle(serPort, turnVelocity, -(currentA/pi)*180);
%           end
%           recordAngleTurn(serPort);
%             
%             tmpX = intersect(find(hitPoints(:,1) < currentX + 0.15), find(hitPoints(:,1) > currentX - 0.15));
%           tmpA = intersect(find(hitPoints(:,2) < currentA + 0.2), find(hitPoints(:,2) > currentA - 0.2));
%             tmpI = intersect(tmpX, tmpA);
% 
%           display('======== HIT POINTS AND currentX ===========')
%           display(hitPoints)
%           display(currentX)
%             display(currentA)
%           display('===========================================')
% 
%             if abs(delta_x) < 0.1
%           %if size(tmpI) > 0
%               display(currentX)
%               display(hitPoints)
%               display('destination impossible: returned to previous position!!!!!!')
%               break
%           end
            
        else % no bump, keep going straight
            
%           display('------------------------------------->going straight');
%             
%           recordRobotTravel(serPort); % update distance traveled
%             display('------------------------------------->currentX GOING STRAIGHT');
%             display(currentX)
%             DistanceSensorRoomba(serPort);
%             AngleSensorRoomba(serPort);
% 
%           SetFwdVelAngVelCreate(serPort, wallVelocity, 0);
%           

        end
        
%         dStart = mapRobot(dStart,drawInterval,fig1, currentX, currentY, currentA);
% 
        pause(0.1);
% 
%       if (abs(goalDistance - currentX) <= margin_error)
%           % robot reached goal!
%           display(currentX)
%           stopRobot();
%           display('Robot reached goal!'); 
%           break
%       end
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


% main function 
function [delta_x, currentA] = followWall(serPort,...
    BumpRight, BumpLeft, BumpFront, oX, oY, currentA, goalDist,...
    fig, drawInterval)

    dStart = tic;
    
    % TODO: reset? sensors to start measuring change
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    fwdVelocity = 0.1;
    turnVelocity = 0.15;
    
    delta_y = 0;
    delta_x = 0; % return: total change in x from starting point
    
    total_offset = 0;
    total_distance = 0;
    
    % margin of error for sensor reading for stop position
    margin_error = power(0.25, 2);
    
    originalWallAngle = 0;
    
    
    travelDist(serPort, 0.05, -0.015);
    
    
    while (~WallSensorReadRoomba(serPort))
        turnAngle(serPort, turnVelocity, 2);
        
        pause(0.1);    
    end
    
    AngleSensorRoomba(serPort);
    turnAngle(serPort, turnVelocity, 66);
    display(AngleSensorRoomba(serPort));
    
    SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
    
    while 1
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if (BumpRight || BumpLeft || BumpFront)
            turnAngle(serPort, turnVelocity, 5);
        end
        if (~WallSensorReadRoomba(serPort))
            
            stopRobot(serPort);
            SetFwdVelRadiusRoomba(serPort, fwdVelocity, -0.2);
        else
            
            SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
        end
        
        pause(0.1);
    end
    
      
%     if (BumpRight)
%         turnAngle(serPort, turnVelocity, 45);
%         originalWallAngle = 45;
%     elseif (BumpLeft)
%         % turn around to hug right
%         turnAngle(serPort, turnVelocity, 120);
%         originalWallAngle = 120;
%     elseif (BumpFront)
%         turnAngle(serPort, turnVelocity, 90);   
%         originalWallAngle = 90;
%     end
%               
%     recordAngleTurn(serPort);

    % Robot turns and re-orients itself depending on bump sensor reading
%     function bumpAction(serPort, BumpRight, BumpLeft, BumpFront)
%         stopRobot();
% 
%         % keep turning until not bumping into wall anymore
%         while (BumpRight || BumpLeft || BumpFront)
% 
%             if (BumpRight)
%                 turnAngle(serPort, turnVelocity, 10);
%             elseif (BumpLeft)
%                 turnAngle(serPort, turnVelocity, -10);
%             elseif (BumpFront)
%                 turnAngle(serPort, turnVelocity, 10);
%             end
% 
%             recordAngleTurn(serPort)
% 
%             [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
% 
%             pause(0.1);
%             display('............TRYING TO ROTATE AWAY...........');
%         end
%         % move forward a bit after turning
%         travelDist(serPort, fwdVelocity, 0.05);
%         display('............LEFT WALL!!!...........');
% 
%     end
%   
%   function recordAngleTurn(serPort)
%       currentA = currentA + AngleSensorRoomba(serPort);
%   end
% 
%   % record robot's distance traveled from last reading
%   function recordRobotTravel(serPort) 
%       recordAngleTurn(serPort);
%       
%       distance = DistanceSensorRoomba(serPort);
%       total_distance = total_distance + distance * 1.02;
%       delta_x = delta_x + distance * cos(currentA);
%       delta_y = delta_y + distance * sin(currentA);
% 
%       disp(['currentA:',num2str(180 * currentA/pi),...
%           ' total_distance:',num2str(total_distance),...
%           ' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y)]);
%       
%       %total_offset = power(delta_x, 2) + power(delta_y, 2);
%       %display(['total_offset: ', num2str(total_offset)]);
%   end
% 
%   function [onM] = on_m_line()
%       
%         display(delta_y);
%         
%         if (abs(delta_y) <= margin_error)
%             onM = 1;
%         else
%             onM = 0;
%         end
%   end
% 
%   
% 
% 
% 
%   % states
%   INIT = 0;
%   DRIVING = 1;
%   state = INIT;
%   while 1
%       [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
%       WallSensor = WallSensorReadRoomba(serPort);
%       %display(WallSensor)
% 
%         if (BumpRight || BumpLeft || BumpFront)
%           bumpAction(serPort, BumpRight, BumpLeft, BumpFront);
% 
%       elseif (WallSensor)
%           % Move Forward
%           SetFwdVelAngVelCreate(serPort, wallVelocity, 0);
%       else % no bump and no wall sensor, went off wall!
%           
%           stopRobot();
%           
%           % try to find wall
%           while (~(WallSensor || BumpRight || BumpLeft || BumpFront))
%               turnAngle(serPort, turnVelocity, -15); % turn right towards wall
%               travelDist(serPort, 0.05, 0.03);
%                 
%               recordRobotTravel(serPort); % update distance traveled
%           
%               [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
%               WallSensor = WallSensorReadRoomba(serPort);
%               
%                 
%                 dStart = mapRobot(dStart,drawInterval,fig,...
%                     oX + delta_x, oY + delta_y, currentA);
%         
%               pause(0.1);
%               display('............TRYING TO FIND WALL...........');
%           end
%           display('............FOUND WALL!!!...........');
%           
%         end
%       
%         dStart = mapRobot(dStart,drawInterval,fig,...
%             oX + delta_x, oY + delta_y, currentA);
%         
%       pause(0.1);
%       
%       recordRobotTravel(serPort); % update distance traveled
%         if ((state == DRIVING) && (on_m_line()))
%             
%             
%             
%             % overshoot
%             if ((currX > goalDist && goalDist > oX) || (currX < goalDist && goalDist < oX))
%                 
%                 
%                 % overshoot, keep going
%                 display('OVVVERRRRR');
% 
%             else
%                 display('RETURNNING');
%                 return
%             end
% 
%             pause(0.1);
%             
%         elseif ((state == INIT) && (total_offset > margin_error))
%             state = DRIVING;
%             display('-----------------------------> first time out of margin');
%         end
%     end
end

function stopRobot(serPort)
    SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
end