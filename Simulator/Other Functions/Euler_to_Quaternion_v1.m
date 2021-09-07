function [q] = Euler_to_Quaternion_v1(pitch,yaw, roll)
%Euler_to_Quaternion version 1 takes yaw, pitch,and roll in radians, and outputs the
%corrosponding quaternion q
%  Takes in pitch, yaw, and roll, IN THAT ORDER, (as opposed to the rest of
%  the code) and outputs the corrosponsing quaternion composed of q1, q2,
%  q3, and q4
q(1) = cos(roll/2)*cos(pitch/2)*cos(yaw/2)-sin(roll/2)*sin(pitch/2)*sin(yaw/2);
q(2) = -sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
q(3) = -cos(roll/2)*sin(pitch/2)*cos(yaw/2)-sin(roll/2)*cos(pitch/2)*sin(yaw/2);
q(4) = -cos(roll/2)*cos(pitch/2)*sin(yaw/2)+sin(roll/2)*sin(pitch/2)*cos(yaw/2);
end

