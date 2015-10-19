
function chaiwen(serPort)

  Initial_Distance = DistanceSensorRoomba(serPort);   % Get the Initial Distance
  disp(['initial_distance: ' num2str(Initial_Distance)]);
  
  Total_Distance = 0;                                 % Initialize Total Distance
    
  state = 'init';
  driveAngle = 2;
  moveDir = 1;
  tStart= tic;
  tInterval = 0.5;
  
  % make roomba hit the wall

  function init()
    fin = 0;
    while (~fin)
      [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers, call ones at beginning, resets timers
      WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
        
      display(WallSensor)
      
      if (BumpFront || BumpRight || BumpLeft)
        fin = 1;
      end
      
%       display(BumpFront)
%       display(BumpRight)
%       display(BumpLeft)

      if (BumpRight)
        SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
        turnAngle(serPort, 0.1, 45);
        fin = 1;
      elseif (BumpLeft)
        SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
        turnAngle(serPort, 0.1, -45);
        fin = 1;
      elseif (BumpFront)
        SetFwdVelRadiusRoomba(serPort, 0, inf); % stop robot
        turnAngle(serPort, 0.1, 90);
        fin = 1;
      else
        SetFwdVelRadiusRoomba(serPort, 0.1, inf);       % Move Forward
        Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);    % Update the Total_Distance covered so far
        display(Total_Distance)
      end
      % pause(0.1);
    end
  end

  function bumpAction(serPort, BumpRight, BumpLeft, BumpFront)
    if (BumpRight)
      %state = 'right';
      SetFwdVelRadiusRoomba(serPort, 0, inf);
      turnAngle(serPort, 0.2, 10);
      moveDir = -1;
    elseif (BumpLeft)
      %state = 'left';
      SetFwdVelRadiusRoomba(serPort, 0, inf);
      turnAngle(serPort, 0.2, -10);
      moveDir = 1;
    elseif (BumpFront)
      %state = 'front';
      SetFwdVelRadiusRoomba(serPort, 0, inf);
      turnAngle(serPort, 0.2, 10);
      moveDir = -1;
    end
    
  end

  init();
  
  display('out of init')
  
  while 1
    [ BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers, call ones at beginning, resets timers
    WallSensor = WallSensorReadRoomba(serPort);      % Read Wall Sensor, Requires WallsSensorReadRoomba file
    % disp([BumpLeft,BumpRight,BumpFront]);
    display(WallSensor)                          % Display WallSensor Value

    %startingDistance = Total_Distance;
    % disp(['robot is: ' state]);

    bumped = BumpRight || BumpLeft || BumpFront;
    display(bumped)

    if (bumped)
      bumpAction(serPort, BumpRight, BumpLeft, BumpFront);
      
    elseif (WallSensor)
      % Move Forward
      Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);
      SetFwdVelRadiusRoomba(serPort, 0.1, inf);
    else
      Total_Distance = Total_Distance + DistanceSensorRoomba(serPort);
      if (toc(tStart) >= tInterval)
        if (strcmp(state,'move1') == 1)
          state = 'move2'
          SetFwdVelRadiusRoomba(serPort, 0, inf);
          turnAngle(serPort, 0.2, 10 * moveDir);
        else
          state = 'move1'
          SetFwdVelRadiusRoomba(serPort, 0.1, driveAngle * moveDir);
        end
        tStart = tic;
      end
    end
    % pause(0.1);
  end
  
  SetFwdVelRadiusRoomba(serPort, 0, 2); % Stop the Robot
%==========================================================================

end
