clc
clear
% This routine tests the point cloud registery with known correspondence

%% Test Point Initialization
% The points are assumed to be on circle with some random noise in their
% radius

theta = 0: 30: 330;
radius = 5 + 1 *(rand(1,length(theta)) - 0.5);

x = radius .* cosd(theta);
y = radius .* sind(theta);
cloudPoints = [x;y];

%% Rotate and Translate point
translationVector = [10;-12];
thetaRot = 25 * pi/180;
R = [cos(thetaRot) -sin(thetaRot);
     sin(thetaRot) cos(thetaRot)];

scenaryPoints = translationVector + R * cloudPoints;

%% Recover the Pose
[recoveredPoistion, recoveredRotation] = findPoseEstimate(cloudPoints,scenaryPoints);
acosd(recoveredRotation(1,1))
plot(x,y,'*')
grid