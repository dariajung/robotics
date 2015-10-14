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
	goalAngle = 0;
	overShoot = 1;	
	
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
	margin_error = power(0.4, 2);

	wallVelocity = 0.2;
	turnVelocity = 0.1;
	
	function recordAngleTurn(serPort)
		total_angle = total_angle + AngleSensorRoomba(serPort);
	end
	% record robot's distance traveled from last reading
	function recordRobotTravel(serPort) 
		recordAngleTurn(serPort);
		
		distance = DistanceSensorRoomba(serPort);
		total_distance = total_distance + distance * .97;
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

	% robot hit wall, reset sensors to start measuring change
	DistanceSensorRoomba(serPort);
	AngleSensorRoomba(serPort);
	
	% STARTING ROBOT
	SetFwdVelRadiusRoomba(serPort, wallVelocity, inf); % Move Forward

	while 1
		[BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
		display('While loop total_x')
		display(total_x)
		
		recordRobotTravel(serPort); % update distance traveled
		display('------------------------------------->TOTAL_X GOING STRAIGHT');
		display(total_x)

		
		if (BumpRight || BumpLeft || BumpFront)
			
			stopRobot();
			
			hitPoints = [hitPoints, total_x];
			[delta_x, theta] = followWall(serPort,BumpRight, BumpLeft, BumpFront, total_x, total_y, fig1, drawInterval);
					
			display('DELTA_X')
			display(delta_x)
			display('THETA')
			display(theta)
		   
			total_x = total_x + overShoot * delta_x;
			total_angle = total_angle + theta;
			
			tmp = intersect(find(hitPoints > total_x + 0.1), find(hitPoints < total_x - 0.1));
			
			display('======== HIT POINTS AND TOTAL_X ===========')
			display(hitPoints)
			display(total_x)
			display('===========================================')

			
			if size(tmp) > 0
				display(total_x)
				display(hitPoints)
				display(tmp)
				display('returned to previous position')
				break
			end
			
			if (abs(delta_x) < 0.1)
				display('robot is stuck inside');
				break
			elseif (total_x > goalDistance)
                display('-------------------------------->THINKS ITS OVERSHOOTING')
				overShoot = -1;
			   
				turnAngle(serPort, turnVelocity, goalAngle + 180 - (theta/pi)*180);
				%display('------------------------------------->finished turnAngle after wall follow');
				%AngleSensorRoomba(serPort); % reset angle sensor to 0
				if (goalAngle == 0)
					goalAngle = 180;
				else
					goalAngle = 0;
				end
			else
				overShoot = 1; % Reset the delta_x sign because we haven't gotten to our goal
				turnAngle(serPort, turnVelocity, goalAngle + -(theta/pi)*180);
			end
			recordAngleTurn(serPort);
			
		else % no bump, keep going straight
			
			display('------------------------------------->going straight');
            
			recordRobotTravel(serPort); % update distance traveled
            display('------------------------------------->TOTAL_X GOING STRAIGHT');
            display(total_x)
            DistanceSensorRoomba(serPort);
            AngleSensorRoomba(serPort);

			SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
			

		end
		
		dElapsed = toc(dStart);
		if (dElapsed > drawInterval)
			mapRobot(fig1, total_x, total_y, total_angle);
			dStart = tic;
		end

		pause(0.1);

		if (abs(goalDistance - total_x) <= margin_error)
			% robot reached goal!
			display(total_x)
			stopRobot();
			display('Robot reached goal!'); 
			break
		end
	end
end