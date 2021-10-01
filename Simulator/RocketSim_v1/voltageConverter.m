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

% Determine maximum force that can be applied by any rotor
omegaMax = min(P.quadParams.cm) * P.quadParams.eamax; 
FMax = min(P.quadParams.kF)*(omegaMax^2);

% Populate conversion matrix
kCVec = P.quadParams.kN./P.quadParams.kF;
G = [ones(1,4); 
     P.quadParams.rotor_loc(2,:);
     -P.quadParams.rotor_loc(1,:);
     -(kTVec').*P.quadParams.omegaRdir];
GInv = inv(G);

% Apply non-negative force and saturation constraints
beta = 0.9;
Fks = min(Fk,4*beta*FMax);
alpha = 1;
NBk2 = rotationMatrix([1;0;0],pi/4)*NBk;
FVec = GInv*[Fks;alpha*NBk2];
while(max(FVec) > FMax)
  alpha = 0.99*alpha;
  FVec = GInv*[Fks;alpha*NBk2];
end
FVec(FVec < 0) = 0;

% Convert force to voltage
omegaVec = sqrt(FVec./P.quadParams.kF);
eak = omegaVec./P.quadParams.cm;