clc; clear; close
Tcurve = load('Cesaroni 15227N2501-P Thrust Points.txt');
% t = linspace(0, 5, 10);
% Thrust = t.^2;
% Tcurve = [t', Thrust'];

% Total simulation time, in seconds
Tsim = Tcurve(end,1);
% Update interval, in seconds.  This value should be small relative to the
% shortest time constant of your system.
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=[0:N-1]'*delt;
Tmat = interpThrust(tVec, Tcurve);

figure(1)
%Plotting thrust vs. simulation time given by interpThrust
plot(Tmat(:,1), Tmat(:,2), 'rx') 
grid on; box on;
hold on
%Plotting thrust vs time given by thrust curve for the selected motor
plot(Tcurve(:,1), Tcurve(:,2), '-b') %
hold off
% xlim([0 6.5])
% xticks([0:0.5:6.5])

%Calculating impulse to compare to actual thrust curve
Impulse = 0;
%Impulse = area under thrust curve [N*s]
for i = 1:length(Tcurve)-1
    %Trapezoidal Area
    base1 = Tcurve(i+1,2);
    base2 = Tcurve(i,2);
    height = Tcurve(i+1,1) - Tcurve(i,1);
    Impulse = Impulse + 0.5*(base1 + base2)*height;
end

% %Calculating impulse given by Tmat to compare to Tcurve
% Impulse2 = 0;
% %Impulse = area under thrust curve [N*s]
% for i = 1:length(Tmat)-1
%     %Trapezoidal Area
%     base1 = Tmat(i+1,2);
%     base2 = Tmat(i,2);
%     height = Tmat(i+1,1) - Tmat(i,1);
%     Impulse2 = Impulse2 + 0.5*(base1 + base2)*height;
% end
% [Impulse; Impulse2]
% (Impulse2/Impulse)*100