function [Xdot] = qrotor_model(X, U)
%% State
% x = [
% x
% y
% z
% u
% v
% w
% phi
% theta
% psi
% p
% q
% r
% ];

%% Input
% u = [
% th1
% th2
% th3
% th4
% ];

%% Constants

% Mass
m = 0.5;

% Gravity
g = 9.81;

% Length of each motor from CoG
L = 0.085;
% r1 = [L L 0.05*L];
% r2 = [-L L 0.05*L];
% r3 = [-L -L 0.05*L];
% r4 = [L -L 0.05*L];

% k_f relates propeller lift to propeller thrust level
k_f = 12;

% k_d relates propeller aerodynamic drag to propeller thrust level
k_d = 0.5;

% C_d relates the total aerodynamic body drag to to velocity
C_d = 0.2;

J_b = [
    1 0 0
    0 1 0
    0 0 1
    ];

invJ_b = inv(J_b);

%% Split up state vector
% Velocity in the body frame
v_b = [
    X(4)
    X(5)
    X(6)
    ];

% Rotation of the body frame wrt the inertial frame
wbi_b = [
    X(10)
    X(11)
    X(12)
    ];

phi = X(7);
theta = X(8);
psi = X(9);

%% Rotation from inertial frame to body frame
Cb2 = [
    cos(psi) sin(psi) 0
    -sin(psi) cos(psi) 0
    0 0 1
    ];

C21 = [
    cos(theta) 0 -sin(theta)
    0 1 0
    sin(theta) 0 cos(theta)
    ];

C1i = [
    1 0 0
    0 cos(phi) sin(phi)
    0 -sin(phi) cos(phi)
    ];

Cbi = Cb2 * C21 * C1i;

%% Sum of Forces
% Motor force in the body frame
Fmotor_b = [
    0
    0
    k_f * (sum(U))
    ];

% Gravity force in the inertial frame
Fgrav_i = [
    0
    0
    -g * m
    ];

% Drag force in the body frame
Fdrag_b = v_b * C_d;

% Change gravity to body frame
Fgrav_b = Cbi * Fgrav_i;

F_b = Fgrav_b + Fmotor_b - Fdrag_b;

%% Sum of Moments
M_b = [
    k_f * sum([-L -L L L] * U)
    k_f * sum([L -L -L L] * U)
    sum([k_d -k_d k_d -k_d] * U)
    ];

%% Acceleration

vb_dot = F_b / m - cross(wbi_b, v_b);

%% Angular Acceleration

wbi_b_dot = invJ_b * (M_b - cross(wbi_b, J_b * wbi_b));

%% Euler Angles

Q = eul2quat([phi, theta, psi], "XYZ");

% Rate of change of quaternion
Omega = [
    0 -wbi_b(1) -wbi_b(2) -wbi_b(3)
    wbi_b(1) 0 wbi_b(3) -wbi_b(2)
    wbi_b(2) -wbi_b(3) 0 wbi_b(1)
    wbi_b(3) wbi_b(2) -wbi_b(1) 0
    ];

Q_dot = 0.5 * Omega * Q';

% Check if zero rotation due to quat2eul function
if norm(Q_dot) == 0
    Q_dot = [1; 0; 0; 0];
end

%% Assemble X_dot

Xdot(1:3) = X(4:6);

Xdot(4:6) = vb_dot;

Xdot(7:9) = quat2eul(Q_dot', "XYZ");

Xdot(10:12) = wbi_b_dot;

Xdot = transpose(Xdot);

end