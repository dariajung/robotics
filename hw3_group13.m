%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 3
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% main function
function hw3_group13(serPort)
    time_period = 0.3;
    tStart= tic;                                        % Time limit marker
    radius_multiplier = 0;
    current_radians = 0;
    current_degrees = 0;

    total_distance = 0;
    delta_x = 0;
    delta_y = 0;
    total_offset = 0;
    radius_incremented = false;
    
    [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    SetFwdVelRadiusRoomba(serPort, 0.1, 0.25 + 0.25 * radius_multiplier)

    % initial state: while not bump, spiral until encounter first obstacle
    while (~BumpRight && ~BumpLeft && ~BumpFront)        
        if toc(tStart) > time_period
            recordRobotTravel(serPort)
        end
        
        display(current_degrees) 
        display(radius_multiplier)
        
        if ((current_degrees < 0) && (current_degrees > -5.0))           
            % make spiraling radius larger
            if radius_incremented == false
                radius_multiplier = radius_multiplier + 1;
                SetFwdVelRadiusRoomba(serPort, 0.1, 0.25 + 0.25 * radius_multiplier)
                radius_incremented = true;
            end
        else 
            radius_incremented = false;
          
        end
    
        
        pause(0.1);
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    end
    
    % obstacle found
    
    
    
    function recordAngleTurn(serPort)
		current_radians = current_radians + AngleSensorRoomba(serPort);
        current_degrees = (current_radians/pi)*180;
	end

	% record robot's distance traveled from last reading
	function recordRobotTravel(serPort) 
		recordAngleTurn(serPort);
		
		distance = DistanceSensorRoomba(serPort);
		total_distance = total_distance + distance;
		delta_x = delta_x + distance * cos(current_radians);
		delta_y = delta_y + distance * sin(current_radians);

		disp(['current_degrees:',num2str(current_degrees),...
			' total_distance:',num2str(total_distance),...
			' delta_x:',num2str(delta_x),' delta_y:',num2str(delta_y)]);
		
		total_offset = power(delta_x, 2) + power(delta_y, 2);
		display(['total_offset: ', num2str(total_offset)]);
	end
end
