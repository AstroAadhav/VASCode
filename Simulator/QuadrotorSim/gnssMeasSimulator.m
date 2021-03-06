function [rPtilde,rCtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rPtilde----- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rCtilde ---- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rCtilde is constrained to satisfy norm(rCtilde) = b, where b is
%              the known distance between the two antennas.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Francis Lara
%+==============================================================================+  

%%%Find the simulated primary antenna position measurement, rPtilde%%%
%Coordinates of the primary antenna, I frame
rPI = S.statek.rI + S.statek.RBI'*(P.sensorParams.rA(:,1));
%Coordinates of the primary antenna, relative to reference antenna, G frame
RIG = Recef2enu(P.sensorParams.r0G);
rPG = (RIG)'*rPI;
%Simulate white Gaussian noise, wp(tk)
RP = RIG'*P.sensorParams.RpL*RIG;
wP = mvnrnd(zeros(3,1), RP);
%Simulated primary antenna coordinates, G frame, [m]
rPtilde = rPG + wP';

%%%Find the simulated secondary antenna relative position measurement, rCtilde%%%
%Coordinates of the secondary antenna, I frame
rsI = S.statek.rI + S.statek.RBI'*(P.sensorParams.rA(:,2));
%Coordinates of the secondary antenna, G frame
rSG = (RIG)'*rsI;
%Coordinates of the secondary antenna, relative to the primary, G frame
rC = rSG - rPG;
%Simulate white Gaussian noise, wp(tk)
rCu = rC/norm(rC);
eps = 10^-8;
RC = ((norm(rC))^2) * (P.sensorParams.sigmaC^2) *(eye(3) - rCu*(rCu')) + eps*eye(3);
wC = mvnrnd(zeros(3,1), RC);
%Simulated secondary antenna coordinates, relative to primary, G frame, [m]
rCtilde = rC + wC';
%Normalize rCtilde such that norm(rCtilde) = norm(rC)
rCtilde = norm(rC)*(rCtilde/norm(rCtilde));




