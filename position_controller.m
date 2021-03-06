function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
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
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains
Kp1 = 17;
Kd1 = 6.6;

Kp2 = 17;
Kd2 = 6.6;

Kp3 = 20;
Kd3 = 9;

% Write code here
m = params.mass;
g = params.gravity;
I = params.inertia;
    
if question == 6.2 || question == 6.3 || question == 6.5
    A =[
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 g*sin(desired_state.rot(3)) g*cos(desired_state.rot(3)) 0 0 0 0 0 0 0;
     0 0 0 -g*cos(desired_state.rot(3)) g*sin(desired_state.rot(3)) 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 (I(2,2)-I(3,3))* desired_state.omega(3)^2 / I(1,1) 0 0 0 0 0 0 (I(2,2)-I(3,3))* desired_state.omega(3)/I(1,1) 0;
     0 0 0 0 (I(1,1)-I(3,3))* desired_state.omega(3)^2 / I(2,2) 0 0 0 0 -(I(1,1)-I(3,3))* desired_state.omega(3)/I(2,2) 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0];
    
    B = [
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     1/m 0 0 0;
     0 1/I(1,1) 0 0;
     0 0 1/I(2,2) 0;
     0 0 0 1/I(3,3)];
    
    Q = diag([30 30 30 1 1 1 5 5 5 1 1 1]);
    R = diag([10 20 30 10]);
    
    [K, S, P] = lqr(A,B,Q,R);
    
    C = [
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
    ];
    
    v = [desired_state.pos;desired_state.rot(3)];
    x= [current_state.pos;current_state.rot;current_state.vel;current_state.omega];
    u = - inv(C / (A - B * K) * B) * v - K * x;
    F = u(1) + m * g;
    acc = [0;0;0]; 
else
    Kp = [Kp1;Kp2;Kp3];
    Kd = [Kd1;Kd2;Kd3];
    acc = desired_state.acc - Kd.*(current_state.vel - desired_state.vel) - Kp.*(current_state.pos - desired_state.pos);
    F = params.mass*(params.gravity + acc(3));   
end

end
