%% Render an image from the geometry and camera position
function [renderedImages,maskMaps,depthMaps,distMaps,normalMaps,albedoMaps,pointMaps] = renderSphere(params)

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

albedoFunc = params.albedoFunc;

% Initialization
renderedImages = zeros([imageSize(1:2) nChannels nCams nLightSources]);
depthMaps = zeros([imageSize(1:2) nCams]);
distMaps = zeros([imageSize(1:2) nCams]);
normalMaps = zeros([imageSize(1:2) 3 nCams]);
albedoMaps = zeros([imageSize(1:2) nChannels nCams]);
pointMaps = zeros([imageSize(1:2) 3 nCams]);
maskMaps = false([imageSize(1:2) nCams]);

% LOOP
for ii = 1:nCams

    % Log
    disp([ 'View : ' num2str(ii) ]);

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
    if displayDebug_
        quiver3(centerCam(1),centerCam(2),centerCam(3),...
            u(1,1),u(2,1),u(3,1),0,'g-','Linewidth',2)
    end

    R = params.R;
    c = params.c;
    x_c = c(1); y_c = c(2); z_c = c(3);

    % Two specific points of the line : z=0 and z=centerCam(3)
    %lambda1 = -U1(3,:)./u(3,:);
    %p1 = lambda1.*u+U1(1:3,:);
    %plot3(p1(1,:),p1(2,:),p1(3,:),'mx')

%     pBar = ProgressBar(nPixels, ...
%     'IsParallel', true, ...
%     'WorkerDirectory', pwd(), ...
%     'Title',[ 'View n°' num2str(ii) ]);
%     pBar.setup([], [], []);

    % Intersection of the Plücker line and the geometry
    tZero = zeros(1,nPixels);
    mask = false(1,nPixels);
    for i = progress(1:nPixels)
        u_x = u(1,i);
        u_y = u(2,i);
        u_z = u(3,i);
        U_x = U1(1,i);
        U_y = U1(2,i);
        U_z = U1(3,i);
        
        delta = (2*u_x*(U_x - x_c) + 2*u_y*(U_y - y_c) + 2*u_z*(U_z - z_c))^2 ...
            - (4*u_x^2 + 4*u_y^2 + 4*u_z^2)*((U_x - x_c)^2 ...
            - R^2 + (U_y - y_c)^2 + (U_z - z_c)^2);

        denomCom = (u_x^2 + u_y^2 + u_z^2);

        t1 = (u_x*x_c - U_y*u_y - U_z*u_z - U_x*u_x + u_y*y_c ...
            + u_z*z_c + (R^2*u_x^2 + R^2*u_y^2 + R^2*u_z^2 ...
            - U_x^2*u_y^2 - U_x^2*u_z^2 + 2*U_x*U_y*u_x*u_y ...
            + 2*U_x*U_z*u_x*u_z - 2*U_x*u_x*u_y*y_c - 2*U_x*u_x*u_z*z_c ...
            + 2*U_x*u_y^2*x_c + 2*U_x*u_z^2*x_c - U_y^2*u_x^2 ...
            - U_y^2*u_z^2 + 2*U_y*U_z*u_y*u_z + 2*U_y*u_x^2*y_c ...
            - 2*U_y*u_x*u_y*x_c - 2*U_y*u_y*u_z*z_c + 2*U_y*u_z^2*y_c ...
            - U_z^2*u_x^2 - U_z^2*u_y^2 + 2*U_z*u_x^2*z_c ...
            - 2*U_z*u_x*u_z*x_c + 2*U_z*u_y^2*z_c - 2*U_z*u_y*u_z*y_c ...
            - u_x^2*y_c^2 - u_x^2*z_c^2 + 2*u_x*u_y*x_c*y_c ...
            + 2*u_x*u_z*x_c*z_c - u_y^2*x_c^2 - u_y^2*z_c^2 ...
            + 2*u_y*u_z*y_c*z_c - u_z^2*x_c^2 - u_z^2*y_c^2)^(1/2))/denomCom;

        t2 = -(U_x*u_x + U_y*u_y + U_z*u_z - u_x*x_c - u_y*y_c ...
            - u_z*z_c + (R^2*u_x^2 + R^2*u_y^2 + R^2*u_z^2 ...
            - U_x^2*u_y^2 - U_x^2*u_z^2 + 2*U_x*U_y*u_x*u_y ...
            + 2*U_x*U_z*u_x*u_z - 2*U_x*u_x*u_y*y_c - 2*U_x*u_x*u_z*z_c ...
            + 2*U_x*u_y^2*x_c + 2*U_x*u_z^2*x_c - U_y^2*u_x^2 ...
            - U_y^2*u_z^2 + 2*U_y*U_z*u_y*u_z + 2*U_y*u_x^2*y_c ...
            - 2*U_y*u_x*u_y*x_c - 2*U_y*u_y*u_z*z_c + 2*U_y*u_z^2*y_c ...
            - U_z^2*u_x^2 - U_z^2*u_y^2 + 2*U_z*u_x^2*z_c ...
            - 2*U_z*u_x*u_z*x_c + 2*U_z*u_y^2*z_c - 2*U_z*u_y*u_z*y_c ...
            - u_x^2*y_c^2 - u_x^2*z_c^2 + 2*u_x*u_y*x_c*y_c ...
            + 2*u_x*u_z*x_c*z_c - u_y^2*x_c^2 - u_y^2*z_c^2 ...
            + 2*u_y*u_z*y_c*z_c - u_z^2*x_c^2 - u_z^2*y_c^2)^(1/2))/denomCom;

        t = [t1,t2];
        pts = t.*u(:,i)+U1(1:3,i);
        [~,argmaxZ] = max(pts(3,:));
        if pts(3,argmaxZ) > params.thrZ && delta >= 0
            tZero(i) = t(argmaxZ);
            mask(i) = 1;

            if displayDebug_ && 0
%                 plot3(pts(1,:), pts(2,:),pts(3,:),...
%                     'b+','Linewidth',1)
%                 plot3(pts(1,argmaxZ), pts(2,argmaxZ),pts(3,argmaxZ),...
%                     'r+','Linewidth',2)
                normal = pts(:,argmaxZ) - c';
                normal = normal/norm(normal);
                quiver3(pts(1,argmaxZ), pts(2,argmaxZ),pts(3,argmaxZ), ...
                    normal(1),normal(2),normal(3),'m-')
                quiver3(pts(1,argmaxZ), pts(2,argmaxZ),pts(3,argmaxZ), ...
                    lightSources(1,1),lightSources(2,1),lightSources(3,1),'c-')
            end

        else
            mask(i) = 0;
        end
        
%         updateParallel([], pwd);
    end
%     pBar.release();
%     clear pBar
    
    points = tZero.*u+U1(1:3,:);
    pointMaps(:,:,:,ii) = reshape(points',imageSize(1),imageSize(2),3);

    % Mask maps
    maskMaps(:,:,ii) = reshape(mask,imageSize(1),imageSize(2));

    % Depth and distance maps
    vecMap = centerCam - points;
    vecMapCam = w2cPose(:,1:3)*vecMap;

    depthMap = abs(vecMapCam(3,:));
    depthMaps(:,:,ii) = reshape(depthMap,imageSize(1),imageSize(2));

    distMap = vecnorm(vecMap);
    distMaps(:,:,ii) = reshape(distMap,imageSize(1),imageSize(2));

    % Normals
    % normals = normalsFunc(points(1,:),points(2,:));
    normals = points - c';
    normals = real(normals);

    normals = normals./vecnorm(normals);
    normalMap = w2cPose(:,1:3)*normals;
    normalMaps(:,:,:,ii) = reshape(normalMap',imageSize(1),imageSize(2),3);
    if displayDebug_
        selectedPixels = find(mask);
        selectedPixels = selectedPixels(1:1:end);
        plot3(points(1,selectedPixels), ...
            points(2,selectedPixels), ...
            points(3,selectedPixels), ...
            'mx','Linewidth',2); % Intersection point
        quiver3(points(1,selectedPixels), ...
            points(2,selectedPixels), ...
            points(3,selectedPixels),...
            normals(1,selectedPixels), ...
            normals(2,selectedPixels), ...
            normals(3,selectedPixels), ...
            4,'g','Linewidth',2); % Normal on this point
        %pause
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
            continue
    end
    albedoMaps(:,:,:,ii) = reshape(albedo,...
        imageSize(1),imageSize(2),nChannels);

    % Rendering
    for jj = 1:nLightSources
        switch lightMode
            case 'directional'
                shading = normals'*lightSources(:,jj);
                shading = max([shading, zeros(size(shading))],[],2);
                renderedImage = lightIntensity*albedo.*shading;
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

% Apply masks
for ii = 1:nChannels
    renderedImages(:,:,ii,:,:) = squeeze(renderedImages(:,:,ii,:,:)).*maskMaps;
    albedoMaps(:,:,ii,:) = squeeze(albedoMaps(:,:,ii,:)).*maskMaps;
end
for ii = 1:3
    normalMaps(:,:,ii,:) = squeeze(normalMaps(:,:,ii,:)).*maskMaps;
end
depthMaps = depthMaps.*maskMaps;
distMaps = distMaps.*maskMaps;
