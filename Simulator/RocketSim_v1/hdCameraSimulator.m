function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
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
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is
%              not visible to the camera (the ray from the feature to the
%              camera center never intersects the image plane), then rx is
%              an empty matrix.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Francis Lara
%+==============================================================================+  

%%%Find image coordinates, camera frame%%%
%Convert feature location to B frame
rXB = S.statek.RBI*(rXI - S.statek.rI);
%Convert feature location to C frame
rXC = P.sensorParams.RCB*(rXB - P.sensorParams.rc);
%Convert C frame to image frame
xIm = [P.sensorParams.K,zeros(3,1)]*[rXC;1];
%Convert image frame to pixel frame
pixelScale = 1/P.sensorParams.pixelSize;
Px = 0.5*pixelScale*P.sensorParams.imagePlaneSize(1);
Py = 0.5*pixelScale*P.sensorParams.imagePlaneSize(2);
xPixel = [pixelScale 0 Px; 0 pixelScale Py; 0 0 1]*xIm;
xC = [xPixel(1)/xPixel(3); xPixel(2)/xPixel(3)];
%Simulate white Gaussian noise
wC = mvnrnd(zeros(2,1), P.sensorParams.Rc);
%Measured coordinates in image plane
xTildeC = xC + wC';
if xTildeC(1)<0 || xTildeC(1)>2*Px || xTildeC(2)<0 || xTildeC(2)>Py
rx = [];
else
rx = xTildeC;
end