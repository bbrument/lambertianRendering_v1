%% Render an image from the geometry and camera position
function [renderedImages,depthMaps,distMaps,normalMaps,albedoMaps,pointMaps] = render(params)

% Options
displayDebug_ = 0;

% Parameters
imageSize = params.imageSize;
nChannels = params.nChannels;

K = params.K;

nCams = params.nCameras;
w2cPoses = params.w2cPoses;
cameraType = params.cameraType;

lightIntensity = params.lightIntensity;
lightSource = params.lightSource;

zFunc = params.zFunc;
normalsFunc = params.normalsFunc;

albedoFunc = params.albedoFunc;

% Initialization
renderedImages = zeros([imageSize(1:2) nChannels nCams]);
depthMaps = zeros([imageSize(1:2) nCams]);
distMaps = zeros([imageSize(1:2) nCams]);
normalMaps = zeros([imageSize(1:2) 3 nCams]);
albedoMaps = zeros([imageSize(1:2) nChannels nCams]);
pointMaps = zeros([imageSize(1:2) 3 nCams]);

% LOOP
parfor ii = 1:nCams

    % Log
    disp([ 'View : ' num2str(ii) ]);

    % Load data
    w2cPose = w2cPoses(:,:,ii);
    if strcmp(cameraType,'ortho')
        orthoMat = diag([1 1 0]);
        projMat = K*[orthoMat*w2cPose(:,1:3) w2cPose(:,4)];
    else
        projMat = K*w2cPose;
    end

    % Pixels
    u = 1:imageSize;
    u = u - 0.5;
    v = u;
    [U,V] = meshgrid(u,v);
    pixels = [U(:)';V(:)';ones(1,length(U(:)))];
    nPixels = length(pixels);

    % Planes on the image
    l1 = cross(pixels,pixels+[1;0;0]);
    l2 = cross(pixels,pixels+[0;1;0]);
    pi1 = reshape(projMat'*l1,4,1,nPixels);
    pi2 = reshape(projMat'*l2,4,1,nPixels);

    % Plücker matrix
    L_dual = pagemtimes(pi1,'none',pi2,'transpose') - pagemtimes(pi2,'none',pi1,'transpose');
    [~,~,V] = pagesvd(L_dual);
    V3 = V(:,3,:); V4 = V(:,4,:);
    L = pagemtimes(V3,'none',V4,'transpose') - pagemtimes(V4,'none',V3,'transpose');

    % Points of the Plücker line
    [U,~,~] = pagesvd(L);
    U1 = squeeze(U(:,1,:)); U2 = squeeze(U(:,2,:));
    zeroU1 = (U1(4,:) < 1e-6); zeroU2 = (U2(4,:) < 1e-6);
    U1(:,zeroU1) = U1(:,zeroU1) + U2(:,zeroU1);
    U2(:,zeroU2) = U1(:,zeroU2) + U2(:,zeroU2);
    U1 = U1./U1(4,:); U2 = U2./U2(4,:);

    % Vector of the Plücker line
    u = U2(1:3,:)-U1(1:3,:);
    u = u./vecnorm(u);

    % Two specific points of the line : z=0 and z=centerCam(3)
    lambda1 = -U1(3,:)./u(3,:);
    p1 = lambda1.*u+U1(1:3,:);

    % Intersection of the Plücker line and the geometry
    tZero = zeros(1,nPixels);
    for i = 1:nPixels
        intersectionFunc = @(t)(zFunc(t*u(1,i)+U1(1,i),t*u(2,i)+U1(2,i))-(t*u(3,i)+U1(3,i)));
        [tZero(i),fVal] = fzero(intersectionFunc,0);

        if displayDebug_
            %plot3([p1(1,i) centerCam(1)],[p1(2,i) centerCam(2)],[p1(3,i) centerCam(3)],'b-')
        end
    end
    points = tZero.*u+U1(1:3,:);
    pointMaps(:,:,:,ii) = reshape(points',imageSize(1),imageSize(2),3);

    % Depth and distance maps
    cameraCenter = -w2cPose(:,1:3)'*w2cPose(:,4);
    vecMap = cameraCenter - points;

    depthMap = abs(vecMap(3,:));
    depthMaps(:,:,ii) = reshape(depthMap,imageSize(1),imageSize(2));

    distMap = vecnorm(vecMap);
    distMaps(:,:,ii) = reshape(distMap,imageSize(1),imageSize(2));

    % Normals
    normals = normalsFunc(points(1,:),points(2,:));
    normals = normals./vecnorm(normals);
    normalMap = w2cPose(:,1:3)*normals;
    normalMaps(:,:,:,ii) = reshape(normalMap',imageSize(1),imageSize(2),3);
    if displayDebug_
        selectedPixels = 1:round(nPixels/1000):nPixels;
        plot3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),'mx','Linewidth',2); % Intersection point
        quiver3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),...
            normals(1,selectedPixels),normals(2,selectedPixels),normals(3,selectedPixels),4,'g','Linewidth',2); % Normal on this point
        pause
    end

    % Albedo
    albedo = albedoFunc(points(1,:),points(2,:));
    albedoMaps(:,:,:,ii) = reshape(albedo',...
        imageSize(1),imageSize(2),nChannels);

    % Rendering
    renderedImage = lightIntensity*albedo'.*(normals'*lightSource);
    renderedImages(:,:,:,ii) = reshape(renderedImage,...
        imageSize(1),imageSize(2),nChannels);

end