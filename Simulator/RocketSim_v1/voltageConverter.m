function [eak] = voltageConverter(Fk,NBk,S,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque and thrust.
%
%
% INPUTS
%
% Fk --------- Commanded total thrust at time tk, in Newtons.
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:
%+==============================================================================+  

% Maximum canard angular displacement
thetaMax = 10*(pi/180); % 10 degrees cw or ccw, converted to radians

% Extract quantities from state vector
rI = S.statek.rI;
vI = S.statek.vI;
RBI = S.statek.RBI;
xI = RBI(1,:)'; %body x-axis, expressed in inertial coordinates

% Determine density and speed o sound
[rho,~,a] = calcAtmoProp(rI(3)/P.constants.distConv);
rho = rho*P.constants.denConv; %Convert density from imperial to SI
a = a*P.constants.distConv; %Convert speed from imperial to SI
mach = abs(norm(vI)/a);

% Determine Cn polynomial fit from sim data based on Mach Number
if and(mach>0.01, mach<2.09)
    Cnfit = P.quadParams.Cnfit(abs(mach-P.quadParams.Cnfit(:,1))<0.005,2:end);
elseif mach>2.09
    Cnfit = P.quadParams.Cnfit(end,2:end);
else
    Cnfit = P.quadParams.Cnfit(1,2:end);
end

% Calculate aerdynamic values
epsilon_vI = 1e-3;
dP = 0;
if(norm(vI) > epsilon_vI)
  vIu = vI/norm(vI);
  fd = (xI'*vIu)*norm(vI)^2;
  dP = 0.5*P.quadParams.Ad*rho*fd;
end

% Populate conversion matrix
K = [-2*(P.quadParams.SMc) 0;
     0 2*(P.quadParams.SMc)];
G = dP*K;
GInv = inv(G);

% Convert Torque in body-frame to canard-frame (+45 deg rotation about xB)
NBk2 = rotationMatrix([1;0;0],pi/4)*NBk;

aVec = abs(GInv*[NBk2(2:3,1)]);
% Determine angular displacement from relationship (quadratic if Cn vs alpha is linear)
theta = zeros(length(aVec),1);
for i = 1:length(aVec)
    poly = [Cnfit,-aVec(i)]; % Coefficient of quadratic polynomial
    q = roots(poly);
    p = min(q(q>0),thetaMax); % 0<theta<thetaMax
    if isreal(p(1)) % Set to 0 if the roots are complex
        theta(i) = p(1);
    else
        theta(i) = 0;
    end
    if norm(vI)<50
        theta(i) = 0;
    end
end
% Current control scheme has canards on opposite sides have the same
% angular displacement to reduce roll and simplify calculation
% theta = [theta1; theta2]
% theta1 = displacement for odd-numbered canards (1, 3)
% theta2 = displacement for even-numbered canards (2, 4)
% Use the sign of NBk2 to determine whether the displacement is CW or CCW
thetac = zeros(4,1);
thetac(1) = sign(NBk2(2))*theta(1);
thetac(2) = sign(NBk2(3))*theta(2);
thetac(3) = -thetac(1);
thetac(4) = -thetac(2);

% Convert angular displacement to voltage
% omegaVec = sqrt(FVec./P.quadParams.kF);
eak = thetac*(180/pi); %currently set to display canard displacement in degrees, not voltage