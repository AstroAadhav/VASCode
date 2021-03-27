clc; clear; close
Tcurve = load('Cesaroni 15227N2501-P Thrust Points.txt');

% Total simulation time, in seconds
Tsim = 4;
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=[0:N-1]'*delt;
Tmat = calcThrust(tVec, Tcurve);

figure(1)
%Plotting thrust vs. simulation time given by calcThrust
plot(Tmat(:,1), Tmat(:,2), 'rx') 
grid on; box on;
hold on
%Plotting thrust vs time given by thrust curve for the selected motor
plot(Tcurve(:,1), Tcurve(:,2), '-bo') %
hold off
% xlim([0 6.5])
% xticks([0:0.5:6.5])

%Calculating impulse to compare to original thrust curve
Impulse = 0;
%Impulse = area under thrust curve [N*s]
for i = 1:length(Tcurve)-1
    %Trapezoidal Area
    base1 = Tcurve(i+1,2);
    base2 = Tcurve(i,2);
    height = Tcurve(i+1,1) - Tcurve(i,1);
    Impulse = Impulse + 0.5*(base1 + base2)*height;
end
