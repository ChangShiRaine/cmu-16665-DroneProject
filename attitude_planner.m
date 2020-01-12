function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here
g = params.gravity;

phi = 1 / g * (desired_state.acc(1) * sin(desired_state.rot(3)) ...
    - desired_state.acc(2) * cos(desired_state.rot(3)));
theta = 1 / g * (desired_state.acc(1) * cos(desired_state.rot(3)) ...
    + desired_state.acc(2) * sin(desired_state.rot(3)));
psi = desired_state.rot(3);

phidot = desired_state.omega(3) * 1 / g * (desired_state.acc(1) * cos(desired_state.rot(3)) ...
    + desired_state.acc(2) * sin(desired_state.rot(3)));
thetadot = desired_state.omega(3) * 1 / g * (- desired_state.acc(1) * sin(desired_state.rot(3)) ...
    + desired_state.acc(2) * cos(desired_state.rot(3)));
psidot = desired_state.omega(3);

rot = [phi; theta; psi];
omega = [phidot; thetadot; psidot];

end

