function [w2cPoses,lightSource] = multiViewLight(cameraGrid,...
    cameraRange,lightGrid,lightRange)

% Options
displayDebug_ = 0;

% Initialization
nCameras = prod(cameraGrid);
nLights = prod(lightGrid);

% Camera positions
h = 5;
if nCameras ~= 1
    x = linspace(-cameraRange,cameraRange,cameraGrid(1));
    y = linspace(-cameraRange,cameraRange,cameraGrid(2));
    [X,Y] = meshgrid(x,y);
    Z = h*ones(cameraGrid);
    centerCam = [X(:),Y(:),Z(:)]';
else
    centerCam = [0,0,h]';
end

% Generate poses
nCamPos = size(centerCam,2);
w2cPoses = zeros(3,4,nCamPos);
for ii = 1:nCamPos
    w2cPoses(:,1:3,ii) = generateCamera(centerCam(:,ii),[0;0;0]);
    w2cPoses(:,4,ii) = centerCam(:,ii);
end

% Light sources
h = 5;
if nLights ~= 1
    x = linspace(-lightRange,lightRange,lightGrid(1));
    y = linspace(-lightRange,lightRange,lightGrid(2));
    [X,Y] = meshgrid(x,y);
    Z = h*ones(lightGrid);
    vectors = [X(:),Y(:),Z(:)]';
    lightSource = vectors./vecnorm(vectors);
else
    vectors = [0,0,h]';
    lightSource = vectors./vecnorm(vectors);
end

if displayDebug_ || 1
    quiver3(zeros(1,nLights),zeros(1,nLights),zeros(1,nLights), ...
        lightSource(1,:),lightSource(2,:),lightSource(3,:),...
        'Linewidth',2);   
end