function [zk] = h_meas(xk,wk,RBIBark,rXIMat,mcVeck,P)
% h_meas : Measurement model for quadcopter.
%
% INPUTS
%
% xk --------- 15x1 state vector at time tk, defined as 
% 
%              xk = [rI', vI', e', ba', bg']'
%
%              where all corresponding quantities are identical to those
%              defined for E.statek in stateEstimatorUKF.m and where e is the
%              3x1 error Euler angle vector defined such that for an estimate
%              RBIBar of the attitude, the true attitude is RBI =
%              dRBI(e)*RBIBar, where dRBI(e) is the DCM formed from the error
%              Euler angle vector e.
%
% wk --------- nz-by-1 measurement noise vector at time tk, defined as
%
%              wk = [wPIk', wCIk', w1C', w2C', ..., wNfkC']'
%
%              where nz = 6 + Nfk*3, with Nfk being the number of features
%              measured by the camera at time tk, and where all 3x1 noise
%              vectors represent additive noise on the corresponding
%              measurements.
%
% RBIBark ---- 3x3 attitude matrix estimate at time tk.
%
% rXIMat ----- Nf-by-3 matrix of coordinates of visual features in the
%              simulation environment, expressed in meters in the I
%              frame. rXIMat(i,:)' is the 3x1 vector of coordinates of the ith
%              feature.
%
% mcVeck ----- Nf-by-1 vector indicating whether the corresponding feature in
%              rXIMat is sensed by the camera: If mcVeck(i) is true (nonzero),
%              then a measurement of the visual feature with coordinates
%              rXIMat(i,:)' is assumed to be made by the camera.  mcVeck
%              should have Nfk nonzero values.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% zk --------- nz-by-1 measurement vector at time tk, defined as
%
%              zk = [rPItilde', rCItildeu', v1Ctildeu', ..., vNfkCtildeu']'
%
%              where rPItilde is the 3x1 measured position of the primary
%              antenna in the I frame, rCItildeu is the 3x1 measured unit
%              vector pointing from the primary to the secondary antenna,
%              expressed in the I frame, and viCtildeu is the 3x1 unit vector,
%              expressed in the camera frame, pointing toward the ith 3D
%              feature, which has coordinates rXIMat(i,:)'.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Francis Lara
%+==============================================================================+  
%Initialize zk
zk = zeros(6,1);
%Get inertial to body frame rotation matrix
e = xk(7:9)';
RBI = euler2dcm(e)*RBIBark;

%Measured position  of the primary antenna, I frame
zk(1:3) = xk(1:3) + RBI'*P.sensorParams.rA(:,1);

%Measured unit vector pointing from the primary to secondary antenna
rCBu = (P.sensorParams.rA(:,2)-P.sensorParams.rA(:,1))/(norm(P.sensorParams.rA(:,2)-P.sensorParams.rA(:,1)));
zk(4:6) = RBI'*rCBu;

%%%Unit vector pointing to each 3D feature%%%
%Camera frame origin
rcI = xk(1:3) + RBI'*P.sensorParams.rc;
for i = 1:length(mcVeck)
    if mcVeck(i)==1
    vCI = rXIMat(i,:)' - rcI;
    v1CIu = vCI/norm(vCI);
    zk = [zk; (RBI'*v1CIu)];
    end
end

%Add measurement noise
zk = zk + wk;