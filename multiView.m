function w2cPoses = multiView(cameraGrid,cameraRange,cameraHeight,pointLookAt)

% Initialization
nCameras = prod(cameraGrid);

% Camera positions
if nCameras ~= 1
    x = linspace(-cameraRange,cameraRange,cameraGrid(1));
    y = linspace(-cameraRange,cameraRange,cameraGrid(2));
    [X,Y] = meshgrid(x,y);
    Z = cameraHeight*ones(cameraGrid);
    centerCam = [X(:),Y(:),Z(:)]';
else
    centerCam = [0,0,cameraHeight]';
end

% Generate poses
nCamPos = size(centerCam,2);
w2cPoses = zeros(3,4,nCamPos);
for ii = 1:nCamPos
    RCam = generateCamera(centerCam(:,ii),pointLookAt');
    w2cPoses(:,1:3,ii) = RCam;
    w2cPoses(:,4,ii) = -RCam * centerCam(:,ii);
end