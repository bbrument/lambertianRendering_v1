function w2cPoses = multiViewKnownCams(cameraPositions,pointsLookAt)

% Generate poses
nCamPos = size(cameraPositions,2);
w2cPoses = zeros(3,4,nCamPos);
for ii = 1:nCamPos
    RCam = generateCamera(cameraPositions(:,ii),pointsLookAt(:,ii));
    w2cPoses(:,1:3,ii) = RCam;
    w2cPoses(:,4,ii) = -RCam * cameraPositions(:,ii);
end