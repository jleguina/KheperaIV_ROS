clear
clc

% Robot Parameters
m = 0.565; % Mass
r = 0.021; % Wheel radius
D = 0.145; % Robot diameter
J = 0.1; % Moment of inertia
F = 0.001; % Friction force

A = [0 0 0 0
    0 0 0 0
    0 0 -F/J 0
    0 0 0 -F/J]

B = [1/m, 1/m, 0, 0;
    -D/2*J, -D/2*J, 0, 0; 
    -r/J, 0, r/J, 0;
    0, -r/J, 0, 1/J]

C = [zeros(2,2), eye(2)]

D = zeros(2,4)

ss(A,B,C,D)
