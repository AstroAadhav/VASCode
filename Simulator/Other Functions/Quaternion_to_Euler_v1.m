function [pitch, yaw, roll] = Quaternion_to_Euler_v1(q)
%Converts a quaternion using the pitch-yaw-roll sequence back to pitch,
%yaw, and roll
%   Pitch-Yaw_Roll Squence
pitch=atan((-2*(q(2)*q(4)+q(1)*q(3)))/(q(1)^2+q(2)^2-q(3)^2+q(4)^2));
yaw = asin(2*(q(2)*q(3)-q(1)*q(4)));
roll= atan((-2*(q(1)*q(2)+q(3)*q(4)))/(q(1)^2-q(2)^2+q(3)^2-q(4)^2));
end

