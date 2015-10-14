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
function hw2_team_13(serPort, goalDistance)
	
	fig1 = figure('Name','robot path'); % draw robot path
	axes('XLim', [-2 8], 'YLim', [-5 5]);
	drawInterval = 0.5; % draw every 0.5 seconds
	dStart = tic;

	hitPoints = [];
	goalDistance = 2;
	
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
	margin_error = power(0.4, 2);

	wallVelocity = 0.2;
	turnVelocity = 0.1;
	
	function recordAngleTurn(serPort)
		currentA = currentA + AngleSensorRoomba(serPort);
	end
	% record robot's distance traveled from last reading
	function recordRobotTravel(serPort) 
		recordAngleTurn(serPort);
		
		distance = DistanceSensorRoomba(serPort);
		total_distance = total_distance + distance * .97;
		currentX = currentX + distance * cos(currentA);
		currentY = currentY + distance * sin(currentA);

		disp(['currentA:',num2str(180 * currentA/pi),...
			' total_distance:',num2str(total_distance),...
			' currentX:',num2str(currentX),' currentY:',num2str(currentY)]);
		
		total_offset = power(currentX, 2) + power(currentY, 2);
		display(['total_offset: ', num2str(total_offset)]);
	end

	function stopRobot()
		SetFwdVelRadiusRoomba(serPort, 0, inf); % Stop the Robot
	end

	% robot hit wall, reset sensors to start measuring change
	DistanceSensorRoomba(serPort);
	AngleSensorRoomba(serPort);
	
	% STARTING ROBOT
	SetFwdVelRadiusRoomba(serPort, wallVelocity, inf); % Move Forward

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
                currentX, currentY, currentA, fig1, drawInterval);
					
			display(['DELTA_X::::::::::::: ', num2str(delta_x)])
			display(['CURRENTANGLE:::::::: ', num2str(currentA)])
		   
			currentX = currentX + delta_x;
			
			tmpX = intersect(find(hitPoints(:,1) < currentX + 0.2), find(hitPoints(:,1) > currentX - 0.2));
			tmpA = intersect(find(hitPoints(:,2) < currentA + 0.2), find(hitPoints(:,2) > currentA - 0.2));
            tmpI = intersect(tmpX, tmpA);

			display('======== HIT POINTS AND currentX ===========')
			display(hitPoints)
			display(currentX)
			display('===========================================')

			if size(tmpI) > 0
				display(currentX)
				display(hitPoints)
				display(tmp)
                % matched a previous position
				display('destination impossible: returned to previous position!!!!!!')
				break
			end
			
			if (currentX > goalDistance)
                display('-------------------------------->THINKS ITS OVERSHOOTING')
			   
				turnAngle(serPort, turnVelocity, 180 - (currentA/pi)*180);
				%display('------------------------------------->finished turnAngle after wall follow');
				%AngleSensorRoomba(serPort); % reset angle sensor to 0
            else
				turnAngle(serPort, turnVelocity, -(currentA/pi)*180);
			end
			recordAngleTurn(serPort);
			
		else % no bump, keep going straight
			
			display('------------------------------------->going straight');
            
			recordRobotTravel(serPort); % update distance traveled
            display('------------------------------------->currentX GOING STRAIGHT');
            display(currentX)
            DistanceSensorRoomba(serPort);
            AngleSensorRoomba(serPort);

			SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
			

        end
		
        dStart = mapRobot(dStart,drawInterval,fig1, currentX, currentY, currentA);

		pause(0.1);

		if (abs(goalDistance - currentX) <= margin_error)
			% robot reached goal!
			display(currentX)
			stopRobot();
			display('Robot reached goal!'); 
			break
		end
	end
end