%% Render an image from the geometry and camera position
function [renderedImages,maskMaps,depthMaps,distMaps,normalMaps,albedoMaps,pointMaps] = render(params)

% Options
displayDebug_ = 0;

% Parameters
imageSize = params.imageSize;
nChannels = params.nChannels;

K = params.K;

nCams = params.nCameras;
w2cPoses = params.w2cPoses;
cameraType = params.cameraType;
if strcmp(cameraType,'ortho')
    orthoScale = params.orthoScale;
end

lightMode = params.lightMode;
lightIntensity = params.lightIntensity;
nLightSources = params.nLightSources;
switch lightMode
    case 'directional'
        lightSources = params.lightSources;
    case 'spherical'
        pointLight = params.pointLight;
end

zFunc = params.zFunc;
normalsFunc = params.normalsFunc;

albedoFunc = params.albedoFunc;

% Initialization
renderedImages = zeros([imageSize(1:2) nChannels nCams nLightSources]);
depthMaps = zeros([imageSize(1:2) nCams]);
distMaps = zeros([imageSize(1:2) nCams]);
normalMaps = zeros([imageSize(1:2) 3 nCams]);
albedoMaps = zeros([imageSize(1:2) nChannels nCams]);
pointMaps = zeros([imageSize(1:2) 3 nCams]);
maskMaps = true([imageSize(1:2) nCams]);

% LOOP
for ii = 1:nCams

    % Log
%     disp([ 'View : ' num2str(ii) ]);

    % Load data
    w2cPose = w2cPoses(:,:,ii);
    RCam = w2cPose(:,1:3);
    tCam = w2cPose(:,4);
    centerCam = -RCam'*tCam;
    if strcmp(cameraType,'ortho')
        tCam(3) = 1;
        orthoScaleMat = K;
        orthoScaleMat(1,1) = orthoScale;
        orthoScaleMat(2,2) = orthoScale;

        orthoMat = diag([1 1 0]);
        projMat = orthoScaleMat*[orthoMat*RCam tCam];
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
    L_dual = pagemtimes(pi1,'none',pi2,'transpose') ...
        - pagemtimes(pi2,'none',pi1,'transpose');
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
    U1_mod_1 = U1(1,:); U1_mod_2 = U1(2,:); U1_mod_3 = U1(3,:);
    u1 = u(1,:); u2 = u(2,:); u3 = u(3,:);
    if displayDebug_
        quiver3(centerCam(1),centerCam(2),centerCam(3),...
            u(1,1),u(2,1),u(3,1),0,'g-','Linewidth',2)
    end

    % Two specific points of the line : z=0 and z=centerCam(3)
    %lambda1 = -U1(3,:)./u(3,:);
    %p1 = lambda1.*u+U1(1:3,:);
    %plot3(p1(1,:),p1(2,:),p1(3,:),'mx')

    pBar = ProgressBar(nPixels, ...
    'IsParallel', true, ...
    'WorkerDirectory', pwd(), ...
    'Title',[ 'View n°' num2str(ii) ]);
    pBar.setup([], [], []);

    % Intersection of the Plücker line and the geometry
    tZeroList = zeros(1,nPixels);
%     for i = progress(1:nPixels)
    parfor i = 1:nPixels
        % lambda = (centerCam(3) - U1(3,i)) / u(3,i);
        % U1_mod = U1(1:3,i) + lambda * u(:,i);
        
        U1_1 = U1_mod_1(i); U1_2 = U1_mod_2(i); U1_3 = U1_mod_3(i);
        u1_i = u1(i); u2_i = u2(i); u3_i = u3(i);

        intersectionFunc = @(t)(zFunc(t*u1_i+U1_1, ...
            t*u2_i+U1_2)-(t*u3_i+U1_3));

        if displayDebug_ && 0
            t_test = -10:0.1:10;
            points_test = t_test.*u(:,i)+U1_mod;
            h1 = plot3(points_test(1,:),points_test(2,:),points_test(3,:),'c+');
            inter_test = intersectionFunc(t_test);
            [~,indtZero_test] = min(abs(inter_test));
            pointsInter_test = points_test(:,indtZero_test);
            h2 = plot3(pointsInter_test(1),pointsInter_test(2),pointsInter_test(3),'m+',...
                'Linewidth',5);
            %pause
            delete(h1)
            delete(h2)
        end
        
        [tZero,~,~] = fzero(intersectionFunc,0,optimset('Display','off'));
        tZeroList(i) = tZero;

        if displayDebug_ && 0
            pointInter = tZero*[u1_i;u2_i;u3_i]+[U1_1;U1_2;U1_3];
            % plot3([pointInter(1) centerCam(1)],[pointInter(2) centerCam(2)],[pointInter(3) centerCam(3)],'b-')
            h1 = plot3(pointInter(1),pointInter(2),pointInter(3),'gx',...
                'Linewidth',5);
            %pause
%             delete(h1)
        end
    
        updateParallel([], pwd);
    end
    pBar.release();
    clear pBar
    
    points = tZeroList.*u+U1(1:3,:);
    pointMaps(:,:,:,ii) = reshape(points',imageSize(1),imageSize(2),3);

    % Depth and distance maps
    vecMap = centerCam - points;
    vecMapCam = w2cPose(:,1:3)*vecMap;

    depthMap = abs(vecMapCam(3,:));
    depthMaps(:,:,ii) = reshape(depthMap,imageSize(1),imageSize(2));

    distMap = vecnorm(vecMap);
    distMaps(:,:,ii) = reshape(distMap,imageSize(1),imageSize(2));

    % Normals
    normals = normalsFunc(points(1,:),points(2,:));
    normals = normals./vecnorm(normals);
    normalMap = w2cPose(:,1:3)*normals;
    normalMaps(:,:,:,ii) = reshape(normalMap',imageSize(1),imageSize(2),3);
    if displayDebug_
%         selectedPixels = 1:round(nPixels/1000):nPixels;
        selectedPixels = randsample(1:nPixels, 1000);
        plot3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),'mx','Linewidth',2); % Intersection point
        quiver3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),...
            normals(1,selectedPixels),normals(2,selectedPixels),normals(3,selectedPixels),4,'g','Linewidth',2); % Normal on this point
        pause
    end

    % Albedo
    switch params.renderType
        case '1'
            albedo = squeeze(albedoFunc(points(1,:),points(2,:)))';
        case 'gray'
            albedo = squeeze(albedoFunc(points(1,:),points(2,:)))';
        case 'rgb'
            albedo = squeeze(albedoFunc(points(1,:),points(2,:)));
        case 'noise'
            albedo = squeeze(albedoFunc(points(1,:),points(2,:)));
        otherwise
            albedo = squeeze(albedoFunc(points(1,:),points(2,:)));
    end
    albedoMaps(:,:,:,ii) = reshape(albedo,...
        imageSize(1),imageSize(2),nChannels);

    % Rendering
    for jj = 1:nLightSources
        switch lightMode
            case 'directional'
                shading = normals'*lightSources(:,jj);
                renderedImage = lightIntensity*albedo.*max([shading, zeros(size(shading))],[],2);
                renderedImages(:,:,:,ii,jj) = reshape(renderedImage,...
                    imageSize(1),imageSize(2),nChannels);
            case 'spherical'
                lightVectors = pointLight' - points;
                normLightVectors = vecnorm(lightVectors);
                lightVectors = lightVectors./normLightVectors;
                sphLightIntensity = lightIntensity ./ (4 * pi * normLightVectors);
                shading = sphLightIntensity.*dot(normals,lightVectors);
                shadingClipped = max([shading', zeros(size(shading'))],[],2);
                renderedImage = albedo.*shadingClipped;
                renderedImages(:,:,:,ii,jj) = reshape(renderedImage,...
                    imageSize(1),imageSize(2),nChannels);
            otherwise
        end
    end
end