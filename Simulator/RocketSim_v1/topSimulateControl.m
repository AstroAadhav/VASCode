% Top-level script for calling simulateQuadrotorControl

% 'clear all' is needed to clear out persistent variables from run to run
clear all; clc;
tic
% Total simulation time, in seconds
Tsim = 20;
% Update interval, in seconds
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=[0:N-1]'*delt;
% Populate reference trajectory
r = 2;
R.tVec = tVec;
R.rIstar = [r*ones(N,1),zeros(N,1),zeros(N,1)];
R.vIstar = [zeros(N,1),zeros(N,1),zeros(N,1)];
R.aIstar = [zeros(N,1),zeros(N,1),ones(N,1)];
% The desired xI points toward the origin. The code below also normalizes
% each row in R.xIstar.
R.xIstar = diag(1./vecnorm(R.rIstar'))*(-R.rIstar);

% Matrix of thrust forces acting on the body, in Newtons, expressed in B
Tcurve = load('Cesaroni 15227N2501-P Thrust Points.txt');
S.distMat = 0*[interpThrust(tVec, Tcurve), zeros(N,1), zeros(N,1)];

% Initial position in m
S.state0.r = [1 1 1]';
% Initial attitude expressed as Euler angles, in radians
pitch = 0*(pi/180);
yaw = 0*(pi/180);
S.state0.e = [0 -pi/2+pitch -pi/2+yaw]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 425]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 5 0]';
% Oversampling factor
S.oversampFact = 2;
% Feature locations in the I frame
S.rXIMat = [];%[0,0,0; 0,0,0.7]; 
% Quadrotor parameters and constants
quadParamsScript;
constantsScript;
sensorParamsScript;
P.quadParams = quadParams; 
P.constants = constants; 
P.sensorParams = sensorParams;

%Run simulation with the inital conditions above
Q = simulateQuadrotorControl(R,S,P);

toc
% S2.tVec = Q.tVec;
% S2.rMat = Q.state.rMat;
% S2.eMat = Q.state.eMat;
% S2.plotFrequency = 20;
% S2.makeGifFlag = false;
% S2.gifFileName = 'testGif.gif';
% S2.bounds=2.5*[-1 1 -1 1 -0.1 1];
% visualizeQuad(S2);

%Plot Altitude
figure(2);clf;
plot(Q.tVec,Q.state.rMat(:,3)); grid on; box on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

%Plot 3D trajectory
figure(3)
plot3(Q.state.rMat(:,1),Q.state.rMat(:,2),Q.state.rMat(:,3))
box on; grid on
xmax = max(max(abs(Q.state.rMat(:,1))), 100);
ymax = max(max(abs(Q.state.rMat(:,2))), 100);
graphmax = sqrt(xmax^2 + ymax^2);
xlim([-graphmax graphmax])
ylim([-graphmax graphmax])
% figure(3);clf;
% psiError = unwrap(n*Q.tVec + pi - Q.state.eMat(:,3));
% meanPsiErrorInt = round(mean(psiError)/2/pi);
% plot(Q.tVec,psiError - meanPsiErrorInt*2*pi);
% grid on;
% xlabel('Time (sec)');
% ylabel('\Delta \psi (rad)');
% title('Yaw angle error');

%Plot Euler Angles
figure(4)
subplot(3,1,1)
plot(Q.tVec,Q.state.eMat(:,1)); grid on; box on;
ylabel('Body Z')
subplot(3,1,2)
plot(Q.tVec,Q.state.eMat(:,2) + pi/2); grid on; box on;
ylabel('Body Y')
subplot(3,1,3)
plot(Q.tVec,Q.state.eMat(:,3) + pi/2); grid on; box on;
ylabel('Body X')

%Body-Axis Visualization
for i = 1:length(Q.state.eMat)
RBI = euler2dcm(Q.state.eMat(i,:));
Viz.x(i,:) = RBI(1,:);
Viz.y(i,:) = RBI(2,:);
Viz.z(i,:) = RBI(3,:);
end
figure(5)
% x-axis = red, y-axis = blue, z-axis = green
h1 = plot3([0 Viz.x(1,1)], [0 Viz.x(1,2)], [0 Viz.x(1,3)], 'r');
hold on
h2 = plot3([0 Viz.y(1,1)], [0 Viz.y(1,2)], [0 Viz.y(1,3)], 'b');
h3 = plot3([0 Viz.z(1,1)], [0 Viz.z(1,2)], [0 Viz.z(1,3)], 'g');
hold off
grid on; box on
%Setting graph limits
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
for i = 1:length(Viz.x)
   tic
   set(h1, 'XData', [0 Viz.x(i,1)], 'YData', [0 Viz.x(i,2)], 'ZData', [0 Viz.x(i,3)]);
   set(h2, 'XData', [0 Viz.y(i,1)], 'YData', [0 Viz.y(i,2)], 'ZData', [0 Viz.y(i,3)]);
   set(h3, 'XData', [0 Viz.z(i,1)], 'YData', [0 Viz.z(i,2)], 'ZData', [0 Viz.z(i,3)]);
   title(['t = ', num2str(Q.tVec(i)), ' sec'])
   drawnow
   dt = toc;
   pause(max((delt/S.oversampFact)-dt,0.001))
end

%Output results
[Apogee, apIndex] = max(Q.state.rMat(:,3));
tApogee = Q.tVec(apIndex);
Apogee = Apogee/constants.distConv;
vMax = max(vecnorm(Q.state.vMat,2,2));
vMax = vMax/constants.distConv;
fileID = fopen('SimOut.txt','w');
fprintf(fileID,'%17s %12f %5s\n','Time to Apogee = ',tApogee, ' [s]');
fprintf(fileID,'%9s %12f %5s\n','        Apogee = ',Apogee, ' [m]');
fprintf(fileID,'%15s %12f %6s\n','  Max Velocity = ',vMax, ' [m/s]');
fclose(fileID);