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
    
    hitPoints = [];
	goalDistance = 1.5;
	goalAngle = 0;
	
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

	% robot hit wall, reset sensors to start measuring change
	DistanceSensorRoomba(serPort);
	AngleSensorRoomba(serPort);
	
	% STARTING ROBOT
	SetFwdVelRadiusRoomba(serPort, wallVelocity, inf); % Move Forward
	%travelDist(serPort, 0.3, 4);
	while 1
		[BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);

		if (BumpRight || BumpLeft || BumpFront)
			
			stopRobot();
            hitPoints = [hitPoints, total_x];
			[delta_x, theta] = followWall(serPort,BumpRight, BumpLeft, BumpFront);
		            
			display(delta_x)
			display(theta)
           

			total_x = total_x + delta_x;
			total_angle = total_angle + theta;
			
            tmp = intersect(find(hitPoints > total_x - 0.1), find(hitPoints < total_x + 0.1));
            
            if size(tmp) > 0
                display(hitPoints)
                display(tmp)
                display('returned to previous position')
                break
            end
            
			if (abs(delta_x) < 0.1)
				display('robot is stuck inside');
				break
			elseif (total_x > goalDistance)
			   
			   turnAngle(serPort, turnVelocity, goalAngle + 180 - (theta/pi)*180);
			   %display('------------------------------------->finished turnAngle after wall follow');
			   %AngleSensorRoomba(serPort); % reset angle sensor to 0
				if (goalAngle == 0)
					goalAngle = 180;
				else
					goalAngle = 0;
				end
			else
				turnAngle(serPort, turnVelocity, goalAngle + -(theta/pi)*180);
			end
			recordAngleTurn(serPort);
			
		else % no bump, keep going straight
			
			display('------------------------------------->going straight');
			recordRobotTravel(serPort); % update distance traveled
			SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
			

		end
		

		pause(0.1);

		if (abs(goalDistance - total_x) <= margin_error)
			% robot reached goal!
			stopRobot();
			display('Robot reached goal!'); 
			break
		end
	end
end