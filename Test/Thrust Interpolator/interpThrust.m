function [Tmat] = calcThrust(tVec, Tcurve)
%   Takes thrust curve data points and interpolates to find thrust at any
%   point in time.
%
% Inputs: 
%     tVec - (N-1) x 1 vector containing all of the simulation times
%
%   Tcurve - matrix containing the data points from a thrust curve, of the
%               form [t(s), Thrust(N)]. Must be linear in between points.
%               Curve is approximately linear if the points are close enough
%               together.
%
% Outputs:
%     Tmat - (N-1) x 2 matrix containing the thrust magnitude at each point
%               in time
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%Assuming linear in between the points%%%
%Linear interpolation; create a line between points (x1, y1) and (x2, y2)
%slope = (y2 - y1) / (x2- x1);
%y = m(x-x1) + y1, domain is [x1, x2]
%Create Tmat
Tmat = zeros(length(tVec), 2);
Tmat(:,1) = tVec;
%Find slope in between each data point
for i = 1:length(Tcurve)-1
   %Max time at which slope from x1 to x2 is valid (x2)
   slope(i,1) = Tcurve(i+1, 1);
   %Calculate slope
   slope(i,2) = (Tcurve(i+1, 2) - Tcurve(i, 2))/...
            (Tcurve(i+1, 1) - Tcurve(i, 1));
end

%Calculate Thrust at each point in time
for i = 2:length(tVec)
   %Thrust is zero after motor burnout
   if tVec(i)>Tcurve(end, 1)
        Tmat(i,2) = 0;
   else
        %Determine which slope to use
        for j = 1:length(slope)
            lessthan(j) =  or(tVec(i) < slope(j, 1), tVec(i) == slope(j, 1));
        end
        index = find(lessthan);
        m = slope(index(1), 2);
        %Calculate thrust based on point-slope formula
        Tmat(i, 2) = m*(tVec(i) - Tcurve(index(1)+1, 1)) + Tcurve(index(1)+1, 2);
   end
end