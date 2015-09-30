%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 1
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
function hw1_team_13(serPort)

    % Read Bumpers, call ONCE at beginning, resets readings
    BumpsWheelDropsSensorsRoomba(serPort);
    WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    
    % Initialize variables keeping track of distance travelled by roomba
    total_distance = 0;                                 
    total_x = 0;
    total_y = 0;
    total_angle = 0;
    total_offset = 0;
    
    % margin of error for sensor reading for stop position
    margin_error = power(0.1, 2);

    fwdVelocity = 0.1;
    wallVelocity = 0.2;
    turnVelocity = 0.1;
    
    % make roomba go forward and hit the wall for the first time
    function init()
        
		SetFwdVelRadiusRoomba(serPort, wallVelocity, inf); % Move Forward
        
        while 1
            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            WallSensor = WallSensorReadRoomba(serPort);
            
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
    
    % Robot turns and re-orients itself depending on bump sensor reading
	function bumpAction(serPort, BumpRight, BumpLeft, BumpFront)
		stopRobot();
        
        % keep turning until not bumping into wall anymore
        while (BumpRight || BumpLeft || BumpFront)

            if (BumpRight)
                turnAngle(serPort, turnVelocity, 10);
            elseif (BumpLeft)
                turnAngle(serPort, turnVelocity, -10);
            elseif (BumpFront)
                turnAngle(serPort, turnVelocity, 10);
            end
            
            recordAngleTurn(serPort)

            [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

            pause(0.1);
            display('............TRYING TO ROTATE AWAY...........');
        end
        % move forward a bit after turning
        travelDist(serPort, fwdVelocity, 0.05);
       
        display('............LEFT WALL!!!...........');
        
    end

    function recordAngleTurn(serPort)
        total_angle = total_angle + AngleSensorRoomba(serPort);
    end

    % record robot's distance traveled from last reading
	function recordRobotTravel(serPort) 
        recordAngleTurn(serPort);
        
        distance = DistanceSensorRoomba(serPort) * .97;
        total_distance = total_distance + distance;
        total_x = total_x + distance * cos(total_angle);
        total_y = total_y + distance * sin(total_angle);

		disp(['total_angle:',num2str(180 * total_angle/pi),...
            ' total_distance:',num2str(total_distance),...
            ' total_x:',num2str(total_x),' total_y:',num2str(total_y)]);
        
        total_offset = power(total_x, 2) + power(total_y, 2);
        display(['total_offset: ', num2str(total_offset)]);
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
        display(WallSensor)

        if (BumpRight || BumpLeft || BumpFront)
            bumpAction(serPort, BumpRight, BumpLeft, BumpFront);

        elseif (WallSensor)
            % Move Forward
            SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
        else % no bump and no wall sensor, went off wall!
            
            stopRobot();
            
            % try to find wall
            while (~(WallSensor || BumpRight || BumpLeft || BumpFront))
                turnAngle(serPort, turnVelocity, -15); % turn right towards wall
                travelDist(serPort, 0.05, 0.03);
                recordRobotTravel(serPort); % update distance traveled
            
                [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                WallSensor = WallSensorReadRoomba(serPort);
                
                pause(0.1);
                display('............TRYING TO FIND WALL...........');
            end
            display('............FOUND WALL!!!...........');
            
        end
        
        pause(0.1);
        
        recordRobotTravel(serPort); % update distance traveled
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