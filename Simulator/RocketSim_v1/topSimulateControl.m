% Top-level script for calling simulateQuadrotorControl or
% simulateQuadrotorEstimationAndControl

% 'clear all' is needed to clear out persistent variables from run to run
clear all; clc;
% Seed Matlab's random number: this allows you to simulate with the same noise
% every time (by setting a nonnegative integer seed as argument to rng) or
% simulate with a different noise realization every time (by setting
% 'shuffle' as argument to rng).
%nseed = 131313;
S = rng('shuffle');

% Assert this flag to call the full estimation and control simulator;
% otherwise, only the control simulator is called
estimationFlag = 1;

%Modified to pull full stack planning data from file
%Matrix with columns t_hist, x_hist, y_hist, vx_hist, vy_hist, ax_hist, ay_hist  
load 'PathData.txt'

% Update interval, in seconds
delt = 0.005;
% Total simulation time, in seconds
Tsim = PathData(end,1) + delt;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=[0:N-1]'*delt;
% Populate reference trajectory
R.tVec = PathData(:,1);
R.rIstar = [PathData(:,2),PathData(:,3),zeros(N,1)];
R.vIstar = [PathData(:,4),PathData(:,5),zeros(N,1)];
R.aIstar = [PathData(:,6),PathData(:,7),zeros(N,1)];
% The desired xI points towards the endpoint. The code below also normalizes
% each row in R.xIstar.
R.xIstar = diag(1./vecnorm(R.rIstar'))*-(R.rIstar-R.rIstar(end,:));
% Matrix of disturbance forces acting on the body, in Newtons, expressed in I
S.distMat = 0*randn(N-1,3);
% Initial position in m
S.state0.r = [0 0 0]';
% Initial attitude expressed as Euler angles, in radians
S.state0.e = [0 0 pi/4]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 0]';
% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.state0.omegaB = [0 0 0]';
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

S2.tVec = Q.tVec;
S2.rMat = Q.state.rMat;
S2.eMat = Q.state.eMat;
S2.plotFrequency = 20;
S2.makeGifFlag = false;
S2.gifFileName = 'testGif.gif';
S2.bounds = [0 11 0 11 -1 1];
visualizeQuad(S2);

figure(2);clf;
plot(Q.tVec,Q.state.rMat(:,3)); grid on; box on;
xlabel('Time (sec)');
ylabel('Vertical (m)');
title('Vertical position of CM'); 

%Plot horizontal position
figure(3);clf;
hold on
plot(Q.state.rMat(:,1), Q.state.rMat(:,2),'b'); 
plot(R.rIstar(:,1), R.rIstar(:,2),'--r'); 
legend('Quadrotor Trajectory', 'Given Trajectory', 'Location', 'northwest')
axis equal; grid on; box on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');
hold off

