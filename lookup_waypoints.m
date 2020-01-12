function [waypoints, waypoint_times] = lookup_waypoints(question)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Write code here

%Sample waypoints for hover trajectory
if question == 2 || question == 6.2
    waypoints = [0 0.1 0.2 0.3; 0 0 0 0; 0.5 0.5 0.5 0.5; 0 0 0 0];
    waypoint_times = [0 2 4 6];
end

if question == 3 || question == 6.3
    waypoints = [0 0 0; 0 0 0; 0 1 0 ; 0 0 0];
    waypoint_times = [0 2 4];
end

% if question == 5 || question == 6.5
%     height = 1;
%     tracking_point = 1.1;
%     waypoints = [0 0 0 0 0; 0 0 0  0 0; 0 height tracking_point height 0 ;0 0 15 0 0];
%     waypoint_times = [0 2 4 6 8];
% end

if question == 5 || question == 6.5
    hover_height = 1;
    tracking_point = 1.1;
    waypoints = [0 0; 0 0; hover_height tracking_point; 0 0];
    waypoint_times = [0 10];
end

% if question == 7
%    waypoints = [0 0 0 0; 0 0 0 0; 1 4 7 10 ;0 0 0 0];
%    waypoint_times = [0 2 4 6]; 
% end 

% if question == 7
%     waypoints = [
%         0 0 0 0 0 0 0 0 0 0 0;
%         0 0 0 0 0 0 0 0 0 0 0;
%         1 2 3 4 5 6 7 8 9 10 10;
%         0 0 0 0 0 0 0 0 0 0 0];
%     waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
% end

if question == 7
    waypoints = [
        0 0 0 0;
        0 0 0 0;
        1 4 7 10;
        0 0 0 0];
    waypoint_times = [0 10 20 30];
end

if question == 7.1
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1   1.1 1.4 2  2.6  3.5  4.6  5.9  7.4 9  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.2
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1   1.5 3  4.5  6   7.5 8  9.5 10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.3
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1   2   4   6   8  10  10  10  10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.4
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1   2.5 5  7.5 10  10  10  10  10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.5
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1  3.3 6.6 10  10  10  10  10  10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.6
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1   5  10  10  10  10  10  10  10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 7.7
    waypoints = [0   0   0   0   0   0   0   0   0   0   0;
                 0   0   0   0   0   0   0   0   0   0   0;
                 1  10  10  10  10  10  10  10  10  10  10;
                 0   0   0   0   0   0   0   0   0   0   0];
    waypoint_times = [0 1 2 3 4 5 6 7 8 9 10];
end

if question == 8  || question == 9
    waypoints = [0 2 0 -2 0 ; 0 1 2 1 0 ; 1 1 1 1 1 ; 0 0 0 0 0];
    waypoint_times = [0 2 4 6 8];
end

end
