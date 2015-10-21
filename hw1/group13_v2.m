
function group13_v2(serPort)

    Initial_Distance = 0;
    
    % Initialize variables keeping track of distance travelled by roomba
    Total_Distance = 0;                                 
    total_x = 0;
    total_y = 0;
    total_angle = 0;

    margin_error = 0.06;
    
    state = 'init';
    driveAngle = 2;
    moveDir = 1;
    tStart = tic;
    tInterval = 0.5;
    bumpDistError = 0.05;
    bumpCounter = 0;
    lastBumpDist = 0;

    fwdVelocity = 0.1;
    wallVelocity = 0.3;
    turnVelocity = 0.1;
    
    % make roomba hit the wall
    function init()
        fin = 0;
        while (~fin)
            [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers, call ones at beginning, resets timers
            WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
                
            display(WallSensor)
            
            if (BumpFront || BumpRight || BumpLeft)
                fin = 1;
                DistanceSensorRoomba(serPort); % Get the Initial Distance when bumping into the wall
                disp(['initial_distance: ' num2str(Initial_Distance)]);
            end
            
            % display(BumpFront)
            % display(BumpRight)
            % display(BumpLeft)

            if (BumpRight)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, 45);
                % fin = 1;
                moveDir = -1;
            elseif (BumpLeft)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, -45);
                % fin = 1;
                moveDir = 1;
            elseif (BumpFront)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, 90);
                % fin = 1;
                moveDir = -1;
            else
                SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);       % Move Forward
                % Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);    % Update the Total_Distance covered so far
                % recordRobotTravel(serPort);
                % display(Total_Distance)
            end
            pause(0.1);
        end
    end

    function bumpAction(serPort, BumpRight, BumpLeft, BumpFront)
        if (BumpRight)
            %state = 'right';
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            turnAngle(serPort, turnVelocity, 10);
            moveDir = -1;
        elseif (BumpLeft)
            %state = 'left';
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            turnAngle(serPort, turnVelocity, -10);
            moveDir = 1;
        elseif (BumpFront)
            %state = 'front';
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            turnAngle(serPort, turnVelocity, 10);
            moveDir = -1;
        end
    end

    % record robot's distance traveled from last bump
    function recordRobotTravel(serPort) 
        total_angle = total_angle + AngleSensorRoomba(serPort);
        distance = DistanceSensorRoomba(serPort);
        Total_Distance = Total_Distance + distance;
        total_x = total_x + distance * cos(total_angle);
        total_y = total_y + distance * sin(total_angle);

        display(total_angle)
        display(Total_Distance)
        display(total_x)
        display(total_y)
    end

    init();
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    Total_Distance = DistanceSensorRoomba(serPort);
    lastBumpDist = Total_Distance;
    
    display('out of init')
    
    while power(total_x, 2) + power(total_y, 2) >= power(Total_Distance * margin_error, 2)
        [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers, call ones at beginning, resets timers
        WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
        % disp([BumpLeft,BumpRight,BumpFront]);
        display(WallSensor)                          % Display WallSensor Value

        %startingDistance = Total_Distance;
        disp(['robot is: ' state]);

        bumped = BumpRight || BumpLeft || BumpFront;
        display(bumped)

        if (bumped)
            recordRobotTravel(serPort);

            if bumpCounter >= 5
                bumpCounter = 0;
                if (Total_Distance - lastBumpDist) <= bumpDistError
                    turnAngle(serPort, turnVelocity, 45);
                    display('stuck here!')
                end

                lastBumpDist = Total_Distance;
            else
                bumpCounter = bumpCounter + 1;
                bumpAction(serPort, BumpRight, BumpLeft, BumpFront);    

                % lastBumpDist = lastBumpDist + DistanceSensorRoomba(serPort)
            end
            % bumpAction(serPort, BumpRight, BumpLeft, BumpFront);    
        elseif (WallSensor)
            % Move Forward
            % Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);
            SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
        else
            recordRobotTravel(serPort);
            % Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);
            if (toc(tStart) >= tInterval)
                if (strcmp(state,'move1') == 1)
                    state = 'move2'
                    SetFwdVelRadiusRoomba(serPort, 0, inf); % stop roomba
                    % turnAngle(serPort, turnVelocity, 10 * moveDir);
                    turnAngle(serPort, turnVelocity, 20 * moveDir);
                else
                    state = 'move1'
                    SetFwdVelRadiusRoomba(serPort, fwdVelocity, driveAngle * moveDir);
                end
                tStart = tic;
            end
        end
        pause(0.1);
    end
    
    SetFwdVelRadiusRoomba(serPort, 0, 2); % Stop the Robot
%==========================================================================

end