function [RCam,centerCam,lightSource] = multiViewLight(nCameras,cameraRange,nLights,lightRange)

% Options
displayDebug_ = 0;

% Camera rotation expressed in the world coordinate system pointing down
R = [  1       0                 0
    0       cos(pi)  -sin(pi)
    0       sin(pi)   cos(pi)  ];

% Camera centers
x = linspace(-cameraRange,cameraRange,sqrt(nCameras));
y = x;
[X,Y] = meshgrid(x,y);
h = 5;
Z = h*ones(sqrt(nCameras));
if displayDebug_
    plot3(X,Y,Z,'b*');
end
centerCam = [X(:),Y(:),Z(:)]';
nPoints = size(centerCam,2);

% Angles
theta = atan(centerCam(2,:)./centerCam(3,:)); % rot around x-axis
phi = atan(centerCam(1,:)./centerCam(3,:)); % rot around y-axis
RCam = zeros(3,3,nPoints);
for i = 1:nPoints
    rotX = [  1       0                 0;
            0       cos(theta(i))  -sin(theta(i));
            0       sin(theta(i))   cos(theta(i))  ];
    rotY = [  cos(phi(i))       0     -sin(phi(i));
            0              1   0;
            sin(phi(i))       0   cos(phi(i))  ];
    RCam(:,:,i) = R*rotX*rotY;

    if displayDebug_
        absPose = rigid3d(RCam(:,:,i),centerCam(:,i)');
        hold on; axis equal;
        plotCamera('AbsolutePose',absPose,'Size',0.1)
        %pause
    end
end

% Light sources
x = linspace(-lightRange,lightRange,sqrt(nLights));
y = x;
[X,Y] = meshgrid(x,y);
h = 5;
Z = h*ones(sqrt(nLights));
vectors = [X(:),Y(:),Z(:)]';
lightSource = vectors./vecnorm(vectors);

if displayDebug_ || 1
    quiver3(zeros(1,nLights),zeros(1,nLights),zeros(1,nLights), ...
        lightSource(1,:),lightSource(2,:),lightSource(3,:));   
end