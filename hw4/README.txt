% Homework 4
%
% Team number: 13
% Team leader: Daria Jung (djj2115)
% Team members:
% Chaiwen Chou (cc3636)
% Joy Pai (jp3113)
% Daria Jung (djj2115)

We wrote our Part 1 in MATLAB.

1. How to run code (copy and paste into MATLAB console):

hw4_team_13(1,'hw4_world_and_obstacles_convex.txt', 'hw4_start_goal.txt');

The main function takes three parameters, serPort (which is unused until the roborace part), the text file for the world and obstacles, and the text file for start and goal.

We output a graphical representation of the world and obstacles, with the visibility graph  in black, obstacles in light blue, and the shortest path in red. The start point is a magenta dot, and the goal is a green dot.

We grew our obstacles by using the right top corner of the robot as the reference point for the reflection algorithm. Because of this we shifted the shortest path down and left by the radius of the robot. 

Reflection Algorithm: Line 177

Visibility graph: 336

Convex Hull: 213

Dijkstra’s Algorithm: 600