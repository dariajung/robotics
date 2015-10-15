%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 2 (mapRobot.m)
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function newTic = mapRobot(tStart, tInterval, f, robot_x, robot_y, r_angle)

    tElapsed = toc(tStart);
    if (tElapsed < tInterval)
        newTic = tStart;
        return
    end
        
    figure(f);
    hold on;
    
    lineLength = 0.7;
    
    if (r_angle > 2 * pi)
        r_angle = r_angle - 2 * pi;
    elseif (r_angle < 2 * pi)
        r_angle = r_angle + 2 * pi;
    end

    line([robot_x robot_x + lineLength * cos(r_angle)],...
        [robot_y robot_y + lineLength * sin(r_angle)],...
        'LineWidth', 1, 'Color', [0, 0.8, 1]);
    plot(robot_x, robot_y, '.r');
    
    newTic = tic;

end