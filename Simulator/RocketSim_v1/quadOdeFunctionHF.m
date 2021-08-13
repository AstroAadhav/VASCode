function [Xdot] = quadOdeFunctionHF(t,X,eaVec,distVec,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     quadrotor dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegaVec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%        omegaVec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegaVec(i) is the angular rate of the ith rotor.
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+

% Extract quantities from state vector
rI = X(1:3);
vI = X(4:6);
RBI = zeros(3,3);
RBI(:) = X(7:15);
omegaB = X(16:18);
omegaVec = X(19:22);

% Determine forces and torques for each rotor from rotor angular rates.  The
% ith column in FMat is the force vector for the ith rotor, in B.  The ith
% column in NMat is the torque vector for the ith rotor, in B.  Note thate
% we negate P.quadParams.omegaRdir because the torque acting on the body is
% in the opposite direction of the angular rate vector for each rotor.
FMat = [zeros(2,4);(P.quadParams.kF.*(omegaVec.^2))'];
NMat = [zeros(2,4);(P.quadParams.kN.*(omegaVec.^2).*(-P.quadParams.omegaRdir)')'];

% Assign some local variables for convenience
mq = P.quadParams.m;
gE = P.constants.g;
Jq = P.quadParams.Jq;
omegaBx = crossProductEquivalent(omegaB);
xI = RBI(1,:)'; %body x-axis, expressed in inertial coordinates

% Determine density and speed o sound
[rho,~,a] = calcAtmoProp(X(3)/P.constants.distConv);
rho = rho*P.constants.denConv; %Convert density from imperial to SI
a = a*P.constants.distConv; %Convert speed from imperial to SI

% Determine CD from simulation data based on Mach number
mach = abs(norm(vI)/a);
if and(mach>0.01, mach<2.09)
    Cd = P.quadParams.Cd(abs(mach-P.quadParams.Cd(:,1))<0.005,2);
elseif mach>2.09
    Cd = P.quadParams.Cd(end,2);
elseif mach<0.01
    Cd = P.quadParams.Cd(1,2);
end

% Determine CD 2nd order polynomial fit from sim data based on Mach Number
if and(mach>0.01, mach<2.09)
    Cnfit = P.quadParams.Cnfit(abs(mach-P.quadParams.Cnfit(:,1))<0.005,2:end);
elseif mach>2.09
    Cnfit = P.quadParams.Cnfit(end,2:end);
elseif mach<0.01
    Cnfit = P.quadParams.Cnfit(1,2:end);
end

% Calculate aerodynamic forces
epsilon_vI = 1e-3;
da = 0; dn = 0; dP = 0; vIu = [0;0;1]; %axial force, normal force, dynamic pressure, inertial velocity unit vector
if(norm(vI) > epsilon_vI)
  vIu = vI/norm(vI);
  fd = (xI'*vIu)*norm(vI)^2;
  dP = 0.5*P.quadParams.Ad*rho*fd;
  da = dP*Cd;
end
vBu = RBI*vIu;
yaw = atan2(vBu(2), vBu(1));
pitch = atan2(vBu(3), sqrt(vBu(1)^2 + vBu(2)^2));
Cn = polyval(Cnfit, abs([0;pitch;yaw]));
% Test = sqrt(0.5*rho*P.quadParams.Ad*Cn)*(P.quadParams.StaticMargin^1.5)
% Find derivatives of state elements
rIdot = vI;
% vIdot = ([0;0;-mq*gE] + RBI'*sum(FMat,2) + RBI'*distVec - da*vIu)/mq;
vIdot = ([0;0;-mq*gE] + RBI'*sum(FMat,2) + RBI'*distVec - RBI'*([da;0;0]))/mq;
RBIdot = -omegaBx*RBI;
NB = sum(NMat,2);

% Corrective Moment
NBc = -(0.5*P.quadParams.Ad*rho*norm(vI)^2)*P.quadParams.StaticMargin*[0;pitch;yaw].*Cn;
% % Damping Moment
NBd = -(0.5*P.quadParams.Ad*rho*norm(vI)).*Cn*(P.quadParams.StaticMargin^2).*omegaB.*[0;1;1];
% Total Moment
NB = NB + NBc + NBd;

omegaBdot = inv(Jq)*(NB - omegaBx*Jq*omegaB);
omegaVecdot = (eaVec.*P.quadParams.cm - omegaVec)./P.quadParams.taum;

% Load the output vector
Xdot = [rIdot;vIdot;RBIdot(:);omegaBdot;omegaVecdot];
