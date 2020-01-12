function trajectory_state = trajectory_planner(question, waypoints, max_iter, waypoint_times, time_step)

% Input parameters
% 
%   question: Which question we are on in the assignment
%
%   waypoints: Series of points in [x; y; z; yaw] format
%
%   max_iter: Number of time steps
%
%   waypoint_times: Time we should be at each waypoint
%
%   time_step: Length of one time_step
%
% Output parameters
%
%   trajectory_state: [15 x max_iter] output trajectory as a matrix of states:
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
%
%************  TRAJECTORY PLANNER ************************

% Write code here

%  * Size: 15 x n where n is the number of time steps
%    * 1:3: position
%    * 4:6: linear velocity
%    * 7:9: orientation
%    * 10:12: angular velocity
%    * 13:15: linear accelerations

% Sample code for hover trajectory
if question == 2 || question == 3 || question == 6.2 || question == 6.3 || question == 6.3
    trajectory_state = zeros(15,max_iter);
    % [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
    
    current_waypoint_number = 1;
    for iter = 1:max_iter
        if (current_waypoint_number<length(waypoint_times))
            if((iter*time_step)>waypoint_times(current_waypoint_number+1))
                current_waypoint_number = current_waypoint_number + 1;
            end
        end
        trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
        trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
    end
end

if question == 4
    % should add variable 'Mode' to switch, but just hard coded here
    % preparation for question 5
    
    % idle
    trajectory_state = zeros(15, max_iter);
    
    % takeoff
    trajectory_state = zeros(15, max_iter);
    desired_height = 5;
    desired_velocity = 1;
    for i = 1:max_iter
        trajectory_state(3, i) = (i-1) * desired_height / (max_iter - 1);
        trajectory_state(6, i) = desired_velocity;
    end
    
    % hover
    trajectory_state = zeros(15, max_iter);
    desired_height = 5;
    for i = 1:max_iter
        trajectory_state(3, i) = desired_height;
    end
    
    % tracking
    sample_rate = 1 / time_step;
    trajectory = waypointTrajectory(waypoints(1:3,:)', ...
        'TimeOfArrival', waypoint_times, ...
        'SampleRate', sample_rate);

    trajectory_state = zeros(15, max_iter);
    
    for i = 1:max_iter
        [position, orientation, velocity, acceleration, angularVelocity] = trajectory();
        trajectory_state(1:3, i) = position;
        trajectory_state(4:6, i) = velocity;
        trajectory_state(7:9, i) = quat2eul(orientation);
        trajectory_state(10:12, i) = angularVelocity;
        trajectory_state(13:15, i) = acceleration;
    end  
    
    % landing
    trajectory_state = zeros(15, max_iter);
    desired_height = 5;
    desired_velocity = -1;
    for i = 1:max_iter
        trajectory_state(3, i) = desired_height - (i-1) * desired_height / (max_iter - 1);
        trajectory_state(6, i) = desired_velocity;
    end 
end

%% Use MATLAB waypointTrajectory API to generate trajectory
% if question == 5
%     sample_rate = 1 / time_step;
%     roll = zeros(size(waypoints(4,:)'));
%     pitch = zeros(size(waypoints(4,:)'));
%     oritentation_eul = [waypoints(4,:)', pitch, roll];
%     trajectory = waypointTrajectory(waypoints(1:3,:)', ...
%         'TimeOfArrival', waypoint_times, ...
%         'SampleRate', sample_rate, ...
%         'Orientation', eul2rotm(oritentation_eul));
%         %'Velocities', waypoints(4:6,:)');
% 
%     trajectory_state = zeros(15, max_iter);
%     
%     for i = 1:max_iter
%         [position, orientation, velocity, acceleration, angularVelocity] = trajectory();
%         trajectory_state(1:3, i) = position;
%         trajectory_state(4:6, i) = velocity;
%         trajectory_state(7:9, i) = rotm2eul(orientation);
%         trajectory_state(10:12, i) = angularVelocity;
%         trajectory_state(13:15, i) = acceleration;
%     end   
% end
% 

%% another way to rise from [0,0,1] to [0,0,1.1]
% if question == 5 || question == 6.5
%     trajectory_state = zeros(15, max_iter);
%     for i = 1:max_iter
%         trajectory_state(3, i) = waypoints(3, 1) + ...
%             (waypoints(3, length(waypoint_times)) - waypoints(3, 1)) * ...
%             (i - 1)/ (max_iter - 1);
%     end 
% end

if question == 5 || question == 6.5
    trajectory_state = zeros(15, max_iter);
    for i = 1:max_iter
        trajectory_state(3, i) = waypoints(3, length(waypoint_times));
        %% heading of 15 degree
        trajectory_state(9, i) = deg2rad(15);
    end 
end
 
if question == 7
    sample_rate = 1 / time_step;
    trajectory = waypointTrajectory(waypoints(1:3,:)', ...
        'TimeOfArrival', waypoint_times, ...
        'SampleRate', sample_rate);
    
    trajectory_state = zeros(15, max_iter);
    
    for i = 1:max_iter
        [position, orientation, velocity, acceleration, angularVelocity] = trajectory();
        trajectory_state(1:3, i) = position;
        trajectory_state(4:6, i) = velocity;
        trajectory_state(7:9, i) = quat2eul(orientation);
        trajectory_state(10:12, i) = angularVelocity;
        trajectory_state(13:15, i) = acceleration;
    end   
end

if question == 8
    x = pi * [0:.5:2]; 
    y = [1 0 2 0 -2 0 1; 
         0 0 1 2  1 0 0];
    pp = spline(x,y);
    yy = ppval(pp, linspace(0,2*pi,(max_iter-1)/4+1));
    
    trajectory_state = zeros(15, max_iter);
    %x
    trajectory_state(1,1:2001) = yy(1,:);
    trajectory_state(1,2001:4001) = yy(1,:);
    trajectory_state(1,4001:6001) = yy(1,:);
    trajectory_state(1,6001:8001) = yy(1,:);
    %y
    trajectory_state(2,1:2001) = yy(2,:);
    trajectory_state(2,2001:4001) = yy(2,:);
    trajectory_state(2,4001:6001) = yy(2,:);
    trajectory_state(2,6001:8001) = yy(2,:);
    %z
    trajectory_state(3,:) = ones(1,max_iter);
    
    %% 1st phase, v = 0->1
    trajectory_state(4,1) = 0;
    trajectory_state(4,2001) = 1;
    for i = 2:2000
        velocity = 1 / 2000 * (i-1);
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
    end
    
    %% 2nd phase, v = 1, fly the circle two times
    trajectory_state(4,2001) = 1;
    trajectory_state(4,6001) = 1;
    for i = 2002:6000
        velocity = 1;
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
    end
    
    %% 3rd phase, v = 1->0
    trajectory_state(4,6001) = 1;
    trajectory_state(4,8001) = 1;
    for i = 6002:8000
        velocity = 1 - 1 / ((max_iter - 1)/4) * (i-6001);
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
    end
    
end


if question == 9
    x = pi * [0:.5:2]; 
    y = [1 0 2 0 -2 0 1; 
         0 0 1 2  1 0 0];
    pp = spline(x,y);
    yy = ppval(pp, linspace(0,2*pi,(max_iter-1)/4+1));
    
    trajectory_state = zeros(15, max_iter);
    %x
    trajectory_state(1,1:2001) = yy(1,:);
    trajectory_state(1,2001:4001) = yy(1,:);
    trajectory_state(1,4001:6001) = yy(1,:);
    trajectory_state(1,6001:8001) = yy(1,:);
    %y
    trajectory_state(2,1:2001) = yy(2,:);
    trajectory_state(2,2001:4001) = yy(2,:);
    trajectory_state(2,4001:6001) = yy(2,:);
    trajectory_state(2,6001:8001) = yy(2,:);
    %z
    trajectory_state(3,:) = ones(1,max_iter);
    
    %% 1st phase, v = 0->1
    trajectory_state(4,1) = 0;
    trajectory_state(4,2001) = 1;
    for i = 2:2000
        velocity = 1 / 2000 * (i-1);
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
    end
    
    %% 2nd phase, v = 1, fly the circle two times
    trajectory_state(4,2001) = 1;
    trajectory_state(4,6001) = 1;
    
    for i = 2002:6000
        velocity = 1;
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
        
        % heading
        psi = atan((trajectory_state(2,i)) / trajectory_state(1,i));
        
        if trajectory_state(1,i) < 0
            psi = psi + pi;
        end
        psi = pi + psi;
%         if psi >= 2 * pi
%             psi = psi - 2 * pi;
%         end
        trajectory_state(9,i) = psi;             
    end
    
    trajectory_state(9,2001) = pi / 2;
    trajectory_state(9,4001) = pi / 2;
    trajectory_state(9,6001) = pi / 2;

    trajectory_state(9,2001) = 0;
    trajectory_state(9,4001) = 0;
    trajectory_state(9,6001) = 0;
    
    %% 3rd phase, v = 1->0
    trajectory_state(4,6001) = 1;
    trajectory_state(4,8001) = 1;
    for i = 6002:8000
        velocity = 1 - 1 / ((max_iter - 1)/4) * (i-6001);
        delta_x = trajectory_state(1,i) - trajectory_state(1,i-1);
        delta_y = trajectory_state(2,i) - trajectory_state(2,i-1);
        trajectory_state(4,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_x;
        trajectory_state(5,i) = velocity / sqrt(delta_x^2 + delta_y^2) * delta_y;
        trajectory_state(9,i) = pi / 2;
    end
    
end

end
