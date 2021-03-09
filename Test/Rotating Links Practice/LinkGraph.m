clc; clear; close;

%Plotting the coordinates for the origin (0, 0), first point (X1, Y1), and
%second point (X2, Y2)
figure(1)
%Red, dotted line with an o marker at each point
h = plot([0 X1(1) X2(1)] ,  [0 Y1(1) Y2(1)], '--or');
%Setting graph limits
xlim([-1.5 1.5])
ylim([-1.5 1.5])

% "Animation"
%Each iteration replaces the coordinates for each point on the same figure and re-draws
%the line
for i = 1:length(phi1)
   set(h, 'XData', [0 X1(i) X2(i)]);
   set(h, 'YData', [0 Y1(i) Y2(i)]);
   drawnow
end