% Top-level script for calling simulateQuadrotorControl or
% simulateQuadrotorEstimationAndControl

% 'clear all' is needed to clear out persistent variables from run to run
clear all; clc;
% Seed Matlab's random number: this allows you to simulate with the same noise
% every time (by setting a nonnegative integer seed as argument to rng) or
% simulate with a different noise realization every time (by setting
% 'shuffle' as argument to rng).
% nseed = 131313;
% S.Seed = rng(nseed);
tic
% S.Seed
% Assert this flag to call the full estimation and control simulator;
% otherwise, only the control simulator is called
estimationFlag = 0;
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
S.state0.v = [0 0 400]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [5 0 0]';
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

if(estimationFlag)
  Q = simulateQuadrotorEstimationAndControl(R,S,P);
else
  Q = simulateQuadrotorControl(R,S,P);
end
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

figure(3)
plot3(Q.state.rMat(:,1),Q.state.rMat(:,2),Q.state.rMat(:,3))
box on; grid on

% figure(3);clf;
% psiError = unwrap(n*Q.tVec + pi - Q.state.eMat(:,3));
% meanPsiErrorInt = round(mean(psiError)/2/pi);
% plot(Q.tVec,psiError - meanPsiErrorInt*2*pi);
% grid on;
% xlabel('Time (sec)');
% ylabel('\Delta \psi (rad)');
% title('Yaw angle error');

% %Plot horizontal position
% figure(3);clf;
% hold on
% plot(Q.state.rMat(:,1), Q.state.rMat(:,2)); 
% axis equal; grid on; box on;
% xlabel('X (m)');
% ylabel('Y (m)');
% title('Horizontal position of CM with X-Axis Direction');

% % Plot body x-axis direction
% xDir.Pos = [];
% j = 1;
% for i = 1:N*S.oversampFact/(2*Tsim):(Tsim*S.oversampFact/delt)
%     CM = Q.state.rMat(i,1:2);
%     RBI = euler2dcm(Q.state.eMat(i,:));
%     xDir.Pos(j,:) = RBI'*[1;0;0];
%     xDir.time(j) = Q.tVec(i);
%     quiver(CM(1), CM(2), xDir.Pos(j,1), xDir.Pos(j,2))
%     j = j+1;
% end

figure(4)
subplot(3,1,1)
plot(Q.tVec,Q.state.eMat(:,1)); grid on; box on;
ylabel('Roll')
subplot(3,1,2)
plot(Q.tVec,Q.state.eMat(:,2) + pi/2); grid on; box on;
ylabel('Pitch')
subplot(3,1,3)
plot(Q.tVec,Q.state.eMat(:,3) + pi/2); grid on; box on;
ylabel('Yaw')

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