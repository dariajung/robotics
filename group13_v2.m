function group13_v2(serPort)

    % Initialize variables keeping track of distance travelled by roomba
	init_distance = 0;
    total_distance = 0;                                 
    total_x = 0;
    total_y = 0;
    total_angle = 0;

    margin_error = 0.06;


	% states
	INIT = 0;
	DRIVING = 1;
	TURNING = 2; 
    state = INIT;

    driveAngle = 2;
    moveDir = 1; % counter clockwise
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
                disp(['initial_distance: ' num2str(init_distance)]);
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
        total_distance = total_distance + distance;
        total_x = total_x + distance * cos(total_angle);
        total_y = total_y + distance * sin(total_angle);

        display(total_angle)
        display(total_distance)
        display(total_x)
        display(total_y)
    end

    init();

	% reset sensors
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    state = INIT;

    display('------------->       out of init')
    
    while 1

		recordRobotTravel(serPort); % update distance traveled

		if power(total_x, 2) + power(total_y, 2) <= power(margin_error, 2) % what about in the beginning? always in this radius!!!
			if (state == DRIVING)
				break
			end
		else
			if (state == INIT)
				state = DRIVING;
				display('-------------> first time out of margin');
			end
		end
        
		[ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers, call ones at beginning, resets timers
        WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
        % disp([BumpLeft,BumpRight,BumpFront]);
        % display(WallSensor)                          % Display WallSensor Value

        bumped = BumpRight || BumpLeft || BumpFront;

        if (bumped)
            if bumpCounter >= 5
                bumpCounter = 0;
                if (total_distance - lastBumpDist) <= bumpDistError
                    turnAngle(serPort, turnVelocity, 45);
                    display('stuck here!')
                end

                lastBumpDist = total_distance;
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
            % Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);
            if (toc(tStart) >= tInterval)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop roomba
                    % turnAngle(serPort, turnVelocity, 10 * moveDir);
                turnAngle(serPort, turnVelocity, 20 * -1);
                    % SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
                tStart = tic;
            end
            SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
        end
        pause(0.1);
    end

	display('back to start!');    
    SetFwdVelRadiusRoomba(serPort, 0, 2); % Stop the Robot
%==========================================================================

end
