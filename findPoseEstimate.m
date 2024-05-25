function [recoveredPoistion, recoveredRotation] = findPoseEstimate(cloudPoints,scenaryPoints)

% This function estimates the pose of scenary point for a given input
% cloudPoints,scenaryPoints. We are looking for the translation vector p
% and the rotation matrix R such that
% min sum (p + R * cloudPoints_i - scenaryPoints_i)' * (p + R * cloudPoints_i - scenaryPoints_i)
% Input: 
% a. cloudPoints: list of cloud points either in R2 or R3
% b. scenaryPoints: list of scenary points either in R2 or R3

% Output: 
% i. recoveredPoistion: translation vector either in R2 or R3
% ii. recoveredRotation: rotation matrix either in R2x2 or R3x3

% We recover the Rotation Matrix and the Translation Vector
pointDimension = size(cloudPoints,1);
numberPoints = size(cloudPoints,2);

% Find the mean of cloudPoints and scenaryPoints
cloudPointsMean = (1/numberPoints) * sum(cloudPoints,2);
scenaryPointsMean = (1/numberPoints) * sum(scenaryPoints,2);

%% Find Data Matrix
dataMatrix = zeros(pointDimension,pointDimension);
for i = 1 : numberPoints
    scenaryToMeanDiff = scenaryPoints(:,i) - scenaryPointsMean;
    cloudToMeanDiff = cloudPoints(:,i) - cloudPointsMean;
    dataMatrix = dataMatrix + scenaryToMeanDiff * cloudToMeanDiff';
end

% find SVD of dataMatrix
[U,~,V] = svd(dataMatrix);

%% Solution
onesVector = ones(1,pointDimension - 1);
D_Recovered = diag([onesVector,det(U*V')]);
recoveredRotation = U * D_Recovered * V';
recoveredPoistion = scenaryPointsMean - recoveredRotation * cloudPointsMean;


