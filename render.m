function [renderedImages,depthMap] = render(zf,normalsFunc,w2cPose,...
    lightSourceTab,albedoFunc,K,repCam,intLight,imageSize)

% Render an image from the geometry and camera position
% Inputs :
% - zf : geometry (function that gives Z coord from X and Y coords)
% - normalsFunc : normals function
% - w2cPose : world tp camera pose
% - lightSourceTab : light sources all considered directional
% - albedoFunc : albedoFunc
% - K : calibration matrix
% - repCam : camera response
% - intLight : light intensity
% - imageSize : height and width of the rendered image
% Outputs :
% - renderedImages : rendered images

% Options
displayDebug_ = 0;

% Projection matrix
projMat = K*w2cPose;

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
for i = 1:nPixels
    %i
    intersectionFunc = @(t)(zf(t*u(1,i)+U1(1,i),t*u(2,i)+U1(2,i))-(t*u(3,i)+U1(3,i)));
    [tZero(i),fVal] = fzero(intersectionFunc,0);

    if displayDebug_
        %plot3([p1(1,i) centerCam(1)],[p1(2,i) centerCam(2)],[p1(3,i) centerCam(3)],'b-')
    end
end
points = tZero.*u+U1(1:3,:);

% Depthmap
cameraCenter = -w2cPose(:,1:3)'*w2cPose(:,4);
depthMap = vecnorm(points - cameraCenter);
depthMap = reshape(depthMap,imageSize,imageSize);

% Normals
normals = normalsFunc(points(1,:),points(2,:));
normals = normals./vecnorm(normals);
if displayDebug_
    selectedPixels = 1:round(nPixels/111):nPixels;
    plot3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),'mx','Linewidth',2); % Intersection point
    quiver3(points(1,selectedPixels),points(2,selectedPixels),points(3,selectedPixels),...
        normals(1,selectedPixels),normals(2,selectedPixels),normals(3,selectedPixels),'g'); % Normal on this point
    pause
end

% Albedo
albedo = albedoFunc(points(1,:),points(2,:));
nColors = size(albedo,1);

% Rendering
for i = 1:size(lightSourceTab,2)
    renderedImage = repCam*intLight*albedo'.*(normals'*lightSourceTab(:,i));
    renderedImages(:,:,:,i) = reshape(uint8(renderedImage),imageSize,imageSize,nColors);
end
