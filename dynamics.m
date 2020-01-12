function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

% Write code here

% * state: quadrotor pose 
%    * Size: 16x1
%    * 1:3: position
%    * 4:6: linear velocity
%    * 7:9: orientation
%    * 10:12: angular velocity
%    * 13:16: motor speeds

m = params.mass;
g = params.gravity;
I = params.inertia;

% Input format of eul2rotm: The default order for Euler angle rotations is "ZYX" 
phi = state(7);
theta = state(8);
psi = state(9);
% eul = [phi, theta, psi];
% 
% bRw = eul2rotm(eul,'XYZ');
% wRb = bRw';

R = [
    cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta),-cos(phi)*sin(psi),cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
    cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta),cos(phi)*cos(psi),sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
    -cos(phi)*sin(theta),sin(phi),cos(phi)*cos(theta)
    ];

% Acceleration
%acc = 1 / m * (R * [0; 0; F] - [0; 0; m * g]);
%acc = 1 / m * (wRb * [0; 0; F] - [0; 0; m * g]);
%acc = 1 / m * (wRb * F - m * g - cross(state(10:12), m * state(4:6)));

% Angular acceleration
%ang_acc = inv(I) * (M - cross(state(10:12), I * state(10:12)));

% Assemble state_dot
state_dot = zeros(16,1);
state_dot(1:3)  = state(4:6);
%state_dot(4:6)  = acc(1:3);
state_dot(4) = g * (theta * cos(psi) + phi * sin(psi));
state_dot(5) = g * (theta * sin(psi) - phi * cos(psi));
state_dot(6) = F / m - g;
state_dot(7:9)  = state(10:12);
%state_dot(10:12) = ang_acc(1:3);
state_dot(10) = M(1) / I(1,1);
state_dot(11) = M(2) / I(2,2);
state_dot(12) = M(3) / I(3,3);
state_dot(13:16) = rpm_motor_dot(1:4);
    
end