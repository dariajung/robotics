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
function [delta_x] = followWall(serPort, BumpRight, BumpLeft, BumpFront)

	fwdVelocity = 0.1;
	wallVelocity = 0.2;
	turnVelocity = 0.1;
	
	delta_y = 0;
	delta_x = 0; % return: total change in x from starting point
	
	total_offset = 0;
	total_angle = 0;
	total_distance = 0;
	
	% margin of error for sensor reading for stop position
	margin_error = power(0.1, 2);
	
    if (BumpRight)
        turnAngle(serPort, turnVelocity, 45);
    elseif (BumpLeft)
        % turn around to hug right
        turnAngle(serPort, turnVelocity, 120);
    elseif (BumpFront)
        turnAngle(serPort, turnVelocity, 90);   
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
		delta_x = delta_x + distance * cos(total_angle);
		delta_y = delta_y + distance * sin(total_angle);

		disp(['total_angle:',num2str(180 * total_angle/pi),...
			' total_distance:',num2str(total_distance),...
			' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y)]);
		
		total_offset = power(delta_x, 2) + power(delta_y, 2);
		display(['total_offset: ', num2str(total_offset)]);
	end

	function [onM] = on_m_line()
		
        onM = 0;
		
	end

	function stopRobot()
		SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
	end

	% TODO: reset? sensors to start measuring change
	DistanceSensorRoomba(serPort);
	AngleSensorRoomba(serPort);

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
        if ((state == DRIVING) && (on_m_line()))
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

