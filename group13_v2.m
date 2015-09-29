function group13_v2(serPort)

    % Read Bumpers, call ONCE at beginning, resets readings
    BumpsWheelDropsSensorsRoomba(serPort);
    WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    
    % Initialize variables keeping track of distance travelled by roomba
    total_distance = 0;                                 
    total_x = 0;
    total_y = 0;
    total_angle = 0;
    total_offset = 0;

    margin_error = power(0.3,2);

    fwdVelocity = 0.1;
    wallVelocity = 0.2;
    turnVelocity = 0.1;
    
    % make roomba go forward and hit the wall for the first time
    function init()
        
		SetFwdVelRadiusRoomba(serPort, wallVelocity, inf); % Move Forward
        
        while 1
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            WallSensor = WallSensorReadRoomba(serPort);
            % disp([BumpLeft BumpFront BumpRight]);
            
            if (BumpRight)
                stopRobot();
                turnAngle(serPort, turnVelocity, 45);
                return
            elseif (BumpLeft)
                stopRobot();
                % turn around to hug right
                turnAngle(serPort, turnVelocity, 120);
                return
            elseif (BumpFront)
                stopRobot();
                turnAngle(serPort, turnVelocity, 90);
                return
            elseif (WallSensor) % start with robot parallel to wall
                return
            end
            pause(0.1); % update simulator screen
        end
    end

	function [d,a] = bumpAction(serPort, BumpRight, BumpLeft, BumpFront)
		stopRobot();

        % try to undo bump sensor
        rotCount = 0;
        dir = 1;
        while (BumpRight || BumpLeft || BumpFront)
            rotCount = rotCount + 1;

            if (BumpRight)
                turnAngle(serPort, turnVelocity, 8);
                dir = 1;
            elseif (BumpLeft)
                turnAngle(serPort, turnVelocity, -8);
                dir = -1;
            elseif (BumpFront)
                turnAngle(serPort, turnVelocity, 8);
                dir = 1;
            end

            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

            pause(0.1);
            display('............TRYING TO ROTATE AWAY...........');
        end
        % move forward a bit after turning
        travelDist(serPort, fwdVelocity, 0.03);
        
        d = 0.03;
        a = rotCount * pi * 8/180 * dir;
        display('............LEFT WALL!!!...........');
        
	end

    % record robot's distance traveled from last reading
	function recordRobotTravel(serPort, dD, dA) 
        %total_angle = total_angle + (AngleSensorRoomba(serPort)+dA)/2;
        total_angle = total_angle + AngleSensorRoomba(serPort);
        
        %distance = (DistanceSensorRoomba(serPort)+dD)/2;
        distance = DistanceSensorRoomba(serPort);
        total_distance = total_distance + distance;
        total_x = total_x + distance * cos(total_angle);
        total_y = total_y + distance * sin(total_angle);

		disp(['a:',num2str(180 * total_angle/pi),...
            ' d:',num2str(total_distance),...
            ' x:',num2str(total_x),' y:',num2str(total_y)]);
        
        total_offset = power(total_x, 2) + power(total_y, 2);
        display(['total_offset: ',num2str(total_offset)]);
    end

    function stopRobot()
        SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
    end

    % STARTING ROBOT
    init();

	% robot hit wall, reset sensors to start measuring change
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    display('-------------------------------------->       out of init')
    
    % states
	INIT = 0;
	DRIVING = 1;
	state = INIT;
	while 1
		[BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        WallSensor = WallSensorReadRoomba(serPort);
        % disp([BumpLeft,BumpRight,BumpFront]);
        % display(WallSensor)

        deltaD = 0;
        deltaA = 0;
        if (BumpRight || BumpLeft || BumpFront)
            
            [deltaD, deltaA] = bumpAction(serPort, BumpRight, BumpLeft, BumpFront);

        elseif (WallSensor)
            % Move Forward
            SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
            deltaD = wallVelocity * 0.1;
        else % no bump and no wall sensor, went off wall!
            
            stopRobot();
            
            % try to find the wall
            rotCount = 0;
            while (~(WallSensor || BumpRight || BumpLeft || BumpFront))
                rotCount = rotCount + 1;
                turnAngle(serPort, turnVelocity, -15);
                travelDist(serPort, 0.05, 0.02);
            
                [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                WallSensor = WallSensorReadRoomba(serPort);
                
                pause(0.1);
                display('............TRYING TO FIND WALL...........');
            end
            display('............FOUND WALL!!!...........');
            % check???
            deltaD = rotCount * 0.03;
            deltaA = rotCount * pi * -10/180;
            
        end
        
        pause(0.1);
        
        
        recordRobotTravel(serPort, deltaD, deltaA); % update distance traveled
        if ((state == DRIVING) && (total_offset <= margin_error))
            % robot returned to origin (offset close to 0)
            break
        elseif ((state == INIT) && (total_offset > margin_error))
            state = DRIVING;
            display('-----------------------------> first time out of margin');
        end

	end

	display('Robot returned back to start!');    
    stopRobot();
end