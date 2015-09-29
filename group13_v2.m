function group13_v2(serPort)

    % Initialize variables keeping track of distance travelled by roomba
	% init_distance = 0;
    total_distance = 0;                                 
    total_x = 0;
    total_y = 0;
    total_angle = 0;

    margin_error = power(0.33,2);
    
    %tStart = tic;
    %tInterval = 0.5;
    %bumpDistError = 0.05;
    %bumpCounter = 0;
    %lastBumpDist = 0;

    fwdVelocity = 0.1;
    wallVelocity = 0.2;
    turnVelocity = 0.1;
    
    
    
    % make roomba hit the wall
    function init()
        
		% Read Bumpers, call once at beginning, resets timers
        BumpsWheelDropsSensorsRoomba(serPort);
        WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file

		SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);       % Move Forward
        
        while 1
            [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
            disp([BumpLeft BumpFront BumpRight]);
            WallSensor = WallSensorReadRoomba(serPort);
            
            if (BumpRight)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, 45);
                return
            elseif (BumpLeft)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, -45);
                return
            elseif (BumpFront)
                SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
                turnAngle(serPort, turnVelocity, 90);
                return
            elseif (WallSensor)
                return
            end
            pause(0.1); % update simulator screen
        end
    end

	function [d,a] = bumpAction(serPort, b_right, b_left, b_front)
		% stop roomba
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        if (b_right)
            turnAngle(serPort, turnVelocity, 10);
            a = 0.174;
        elseif (b_left)
            turnAngle(serPort, turnVelocity, -10);
            a = -0.174;
        elseif (b_front)
            turnAngle(serPort, turnVelocity, 10);
            a = 0.174;
        end
		% move forward a bit after turning
        travelDist(serPort, fwdVelocity, 0.03);
        d = 0.03;
        %SetFwdVelRadiusRoomba(serPort, fwdVelocity, inf);
	end

    % record robot's distance traveled from last bump
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
	end

    init();

	% hit wall, reset sensors to start measuring change
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);

    display('-------------------------------------->       out of init')
    
    % states
	INIT = 0;
	DRIVING = 1;
	%TURNING = 2; 
	state = INIT;
	while 1
		[ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        WallSensor = WallSensorReadRoomba(serPort);
        % disp([BumpLeft,BumpRight,BumpFront]);
        % display(WallSensor)

        deltaD = 0;
        deltaA = 0;
        if (BumpRight || BumpLeft || BumpFront)
            %[deltaD, deltaA] = bumpAction(serPort, BumpRight, BumpLeft, BumpFront);
           
            SetFwdVelRadiusRoomba(serPort, 0, inf); % stop roomba

            % try undo bump sensor
            rotCount = 0;
            dir = 1;
            while (BumpRight || BumpLeft || BumpFront)
                rotCount = rotCount + 1;
                
                if (BumpRight)
                    turnAngle(serPort, turnVelocity, 10);
                    dir = 1;
                elseif (BumpLeft)
                    turnAngle(serPort, turnVelocity, -10);
                    dir = -1;
                elseif (BumpFront)
                    turnAngle(serPort, turnVelocity, 10);
                    dir = 1;
                end
            
                [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                
                pause(0.1);
                display('............TRYING TO ROTATE AWAY...........');
            end
            % move forward a bit after turning
            travelDist(serPort, fwdVelocity, 0.03);
            deltaD = 0.03;
            deltaA = rotCount * pi * 10/180 * dir;
            display('............LEFT WALL!!!...........');

        elseif (WallSensor)
            % Move Forward
            SetFwdVelRadiusRoomba(serPort, wallVelocity, inf);
            deltaD = wallVelocity * 0.1;
        else % no bump and no wall sensor, went off wall!
            
            SetFwdVelRadiusRoomba(serPort, 0, inf); % stop roomba
            
            % try to find the wall
            rot = -10;
            rotMult = 1;
            while (~(WallSensor || BumpRight || BumpLeft || BumpFront))
                turnAngle(serPort, turnVelocity, rot * rotMult);
                travelDist(serPort, fwdVelocity, 0.03);
            
                [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                WallSensor = WallSensorReadRoomba(serPort);
                
                pause(0.1);
                display('............TRYING TO FIND WALL...........');
                rotMult = rotMult + 1;
            end
            display('............FOUND WALL!!!...........');
            % check???
            deltaD = rotMult * 0.03;
            deltaA = rotMult * pi * -10/180;
            
        end
        
        pause(0.1);
        
        
        recordRobotTravel(serPort, deltaD, deltaA); % update distance traveled
        totalOffset = power(total_x, 2) + power(total_y, 2);
        display(['totaloff: ',num2str(totalOffset)]);
        if (totalOffset <= margin_error)
            if (state == DRIVING)
                break
            end
        elseif (state == INIT)
            state = DRIVING;
            display('-----------------------------> first time out of margin');
        end

	end

	display('back to start!');    
    SetFwdVelRadiusRoomba(serPort, 0, 2); % Stop the Robot
end