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
S.state0.r = [1 1 100]';
% Initial attitude expressed as Euler angles, in radians
pitch = 10*(pi/180);
yaw = 0*(pi/180);
S.state0.e = [0 -pi/2+pitch -pi/2+yaw]';
% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.v = [0 0 425]';
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

%Run simulation with the inital conditions above
Q = simulateQuadrotorControl(R,S,P);
toc

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

% Mach Graph
figure(5)
mach = zeros(length(Q.tVec),1);
for i = 1:length(Q.tVec)
[rho,~,a] = calcAtmoProp(Q.state.rMat(i,3)/constants.distConv);
rho = rho*constants.denConv; %Convert density from imperial to SI
a = a*constants.distConv; %Convert speed from imperial to SI
mach(i) = abs(norm(Q.state.vMat(i,:))/a);
end
plot(Q.tVec, mach, 'r')
title('Mach Graph')
xlabel('Time [s]')
ylabel('Mach#')
box on; grid on;

%Output results
[Apogee, apIndex] = max(Q.state.rMat(:,3));
tApogee = Q.tVec(apIndex);
Apogee = Apogee/constants.distConv;
vMax = max(vecnorm(Q.state.vMat,2,2));
vMax = vMax/constants.distConv;
fileID = fopen('SimOut.txt','w');
fprintf(fileID,'%17s %12f %5s\n','Time to Apogee = ',tApogee, ' [s]');
fprintf(fileID,'%9s %12f %5s\n','        Apogee = ',Apogee, ' [ft]');
fprintf(fileID,'%15s %12f %6s\n','  Max Velocity = ',vMax, ' [ft/s]');
fclose(fileID);

% Body-Axis Visualization
S2.tVec = Q.tVec;
S2.eMat = Q.state.eMat;
S2.tStep = delt/S.oversampFact;
S2.bounds=[-1 1 -1 1 -1 1];
visualizeAxes(S2);