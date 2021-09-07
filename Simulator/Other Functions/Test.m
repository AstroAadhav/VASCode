% Use this script to test any of the functions in this folder
% Modify any part of this code
clc; clear; close;

rng(131313) % Setting the rng seed will give you the same random numbers every time you run the script
n = 10^3;
Roll = 2*pi*rand(n,1)-pi; % rand(n,1) gives a vector of n random numbers from [0,1], setting limits
Pitch = 2*pi*rand(n,1)-pi;
Yaw = pi*rand(n,1)-pi/2;

for i = 1:n
Quaternion = Euler_to_Quaternion_v1(Pitch(i,1), Yaw(i,1), Roll(i,1));
[Pitch(i,2), Yaw(i,2), Roll(i,2)] = Quaternion_to_Euler_v1(Quaternion);
% Euler = [Roll, Pitch, Yaw]; % [Roll, Pitch, Yaw]
end