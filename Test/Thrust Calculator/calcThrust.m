function [Tmat] = calcThrust(tVec, Tcurve)


%%%Assuming linear in between the points%%%
%Linear interpolation; create a line between points (x1, y1) and (x2, y2)
%slope = (y2 - y1) / (x2- x1);
%y = m(x-x0) + y0
%Create Tmat
Tmat = zeros(length(tVec), 2);
Tmat(:,1) = tVec;
for i = 1:length(Tcurve)-1
   slope(i,1) = Tcurve(i+1, 1);
   slope(i,2) = (Tcurve(i+1, 2) - Tcurve(i, 2))/...
            (Tcurve(i+1, 1) - Tcurve(i, 1));
end

%Determine which slope to use
for i = 2:length(tVec)
   if tVec(i)>Tcurve(end, 1)
        Tmat(i,2) = 0;
   else
        for j = 1:length(slope)
            lessthan(j) =  or(tVec(i) < slope(j, 1), tVec(i) == slope(j, 1));
        end
        index1 = find(lessthan);
        index2 = index1(1);
        m = slope(index2, 2);
        Tmat(i, 2) = m*(tVec(i) - Tcurve(index2+1, 1)) + Tcurve(index2+1, 2);
   end
end