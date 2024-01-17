%% Function to set parameters
function params = setParameters()

% Camera parameters
params.factor = 1; % downscale factor
params.imageSize = [1000 1000]/params.factor; % rendered image size
params.nChannels = 1; % number of channels of the rendered image
params.renderType = 'rgb'; % '1' for an homogeneous albedo, 'gray' for gray-scaled one or 'rgb' for an colored one
if strcmp(params.renderType,'rgb') || strcmp(params.renderType,'noise') % if renderType is 'rgb', update imageSize and nChannels
    params.imageSize = [params.imageSize 3];
    params.nChannels = 3;
end

% Camera intrinsics
px = 500/params.factor; % principal point
py = 500/params.factor;
f = 1000/params.factor; % focal length (default:2000/...)
params.K = [f 0 px ; 0 f py ; 0 0 1];

% params.K = [500 0 px ; 0 500 py ; 0 0 1];

% Camera poses (generated on a grid pointing at the center of the scene)
params.cameraGrid = [3 3]; % number of cameras along x and y
params.cameraRange = 1; % range along both axis (spherePS:20)
params.cameraHeight = 5; % (spherePS:95)
params.pointLookAt = [0,0,-5];%[0.1,-0.2,2.8]; %[0,0,-5] %(spherePS:[0,0,0])
params.w2cPoses = multiView(params.cameraGrid,params.cameraRange,...
    params.cameraHeight,params.pointLookAt); % cameras poses (world-to-camera matrices)
% pixelDisplacement = 100.5;
% depthGT = 5;
% cameraDisplacement = pixelDisplacement/f*depthGT;
% params.cameraPositions = [-1,-1,-1,0,0,0,1,1,1;
%                    -1,0,1,-1,0,1,-1,0,1;
%                    depthGT*ones(1,9)];
% params.cameraPositions(1:2,:) = cameraDisplacement*params.cameraPositions(1:2,:);
% params.pointsLookAt = params.cameraPositions;
% params.pointsLookAt(3,:) = zeros(1,9);
% params.w2cPoses = multiViewKnownCams(params.cameraPositions,params.pointsLookAt); % cameras poses (world-to-camera matrices)
params.nCameras = size(params.w2cPoses,3); % number of cameras

% World-to-camera matrix (w2c) -> Camera-to-world matrix (c2w)
params.c2wPoses = zeros(size(params.w2cPoses));
params.c2wPoses(:,1:3,:) = pagetranspose(params.w2cPoses(:,1:3,:));
params.c2wPoses(:,4,:) = -pagemtimes(params.w2cPoses(:,1:3,:),'transpose',...
    params.w2cPoses(:,4,:),'None');

% Orthographic or perspective camera?
% params.cameraType = 'ortho'; % 'persp' or 'ortho'
params.cameraType = 'persp'; % 'persp' or 'ortho'
if strcmp(params.cameraType,'ortho')
    params.orthoScale = 1;%200/params.factor;
end

% Light source parameters
params.lightMode = 'directional'; % 'directional' or 'spherical'
switch params.lightMode
    case 'directional'
        params.lightIntensity = 1; % light intensity
        params.nLightSources = 1;
        theta = deg2rad(45);
        psi = deg2rad(45);
        angles = [0 theta theta theta 0 0 -theta -theta -theta;
            0 psi 0 -psi psi -psi psi 0 -psi];
        params.lightSources = [cos(angles(1,:)).*sin(angles(2,:));
            -sin(angles(1,:));
            cos(angles(1,:)).*cos(angles(2,:))];

        % for pyramid
        theta = deg2rad(30);
        psi = deg2rad(30);
        angles = [theta; psi];
        params.lightSources = [cos(angles(1))*sin(angles(2));
            -sin(angles(1));
            cos(angles(1))*cos(angles(2))];
    case 'spherical'
        params.nLightSources = 1;
        params.pointLight = [5 5 10];
        params.lightIntensity = 112; % light intensity
    otherwise
end

% Scene parameters
params.geometryType = 'pyramid';
switch params.geometryType
    case 'gaussian'
        mu = [0.2 0.4];
        sigma = [0.7 0.6];
        
        params.zFunc = @(X,Y) 1/(sigma(1)*sigma(2)*sqrt(2*pi))...
            .*exp(-1/2*((X-mu(1)).^2/sigma(1)^2 + (Y-mu(2)).^2/sigma(2)^2));
        
        params.normalsFunc = @(X,Y) [-(2^(1/2).*exp(-(mu(1) - X).^2/(2*sigma(1)^2) - (mu(2)-Y).^2/(2*sigma(2)^2)).*(2*mu(1) - 2*X))/(4*pi^(1/2)*sigma(1)^4);
            -(2^(1/2).*exp(- (mu(1) - X).^2/(2*sigma(1)^2) - (mu(2) - Y).^2/(2*sigma(2)^2)).*(2*mu(2) - 2*Y))/(4*pi^(1/2)*sigma(1)^2*sigma(2)^2);
            ones(1,size(X,2))];% (calculated analytically : cross product of the x and y gradient components of the zFunc)
        params.geomRange = [-10 10]; %multi-gaussian;

    case 'multi-gaussian'
        scale = 15;
        mu_1 = [-1.9 -1.6];
        sigma_1 = [1.8 2];
        mu_2 = [1.8 2.4];
        sigma_2 = [1.5 2.1];
        
        params.zFunc = @(X,Y) (2^(1/2)*scale.*exp(- (mu_1(1) - X).^2/(2*sigma_1(1)^2) ...
            - (mu_1(2) - Y).^2/(2*sigma_1(2)^2)))/(2*pi^(1/2)*sigma_1(1)*sigma_1(2)) ...
            + (2^(1/2)*scale.*exp(- (mu_2(1) - X).^2/(2*sigma_2(1)^2) ...
            - (mu_2(2) - Y).^2/(2*sigma_2(2)^2)))/(2*pi^(1/2)*sigma_2(1)*sigma_2(2));
        
        params.normalsFunc = @(X,Y) [(2^(1/2)*scale.*exp(- (X - mu_1(1)).^2/(2*sigma_1(1)^2) ...
            - (Y - mu_1(2)).^2/(2*sigma_1(2)^2)).*(2*X - 2*mu_1(1)))/(4*pi^(1/2)*sigma_1(1)^3*sigma_1(2)) ...
            + (2^(1/2)*scale.*exp(- (X - mu_2(1)).^2/(2*sigma_2(1)^2) ...
            - (Y - mu_2(2)).^2/(2*sigma_2(2)^2)).*(2*X - 2*mu_2(1)))/(4*pi^(1/2)*sigma_2(1)^3*sigma_2(2));
            (2^(1/2)*scale.*exp(- (X - mu_1(1)).^2/(2*sigma_1(1)^2) ...
            - (Y - mu_1(2)).^2/(2*sigma_1(2)^2)).*(2*Y - 2*mu_1(2)))/(4*pi^(1/2)*sigma_1(1)*sigma_1(2)^3) ...
            + (2^(1/2)*scale.*exp(- (X - mu_2(1)).^2/(2*sigma_2(1)^2) ...
            - (Y - mu_2(2)).^2/(2*sigma_2(2)^2)).*(2*Y - 2*mu_2(2)))/(4*pi^(1/2)*sigma_2(1)*sigma_2(2)^3);
            ones(1,size(X,2))];% (calculated analytically : cross product of the x and y gradient components of the zFunc)
        params.geomRange = [-10 10]; %multi-gaussian;

    case 'sphere'
        if strcmp(params.cameraType, 'persp')
%             params.R = params.cameraHeight*60/100;
            params.R = 5; %(spherePS:5)
        else
            params.R = params.cameraHeight*95/100;
        end
%         params.c = [1 0.5 0]; %centered sphere
        params.c = [-35 -35 5]; %bottom-left sphere %(spherePS:5)
        params.thrZ = params.R*sqrt(1-1^2); 
        params.zFunc = @(X,Y) sqrt(params.R^2 - (X-params.c(1)).^2 - (Y-params.c(2)).^2) + params.c(3);
        params.geomRange = [-50 50]; %(spherePS:[-30 30])
    
    case 'plane'
        theta = deg2rad(0); phi = deg2rad(0);
        planeNormal = [sin(phi)*cos(theta),-sin(theta),cos(phi)*cos(theta)];
        d = 0;
        planeNormal = planeNormal/norm(planeNormal);
        params.zFunc = @(X,Y) -(planeNormal(1)*X+planeNormal(2)*Y+d)/planeNormal(3);
        params.normalsFunc = @(X,Y) [planeNormal(1)/planeNormal(3)*ones(size(X));
            planeNormal(2)/planeNormal(3)*ones(size(X));
            ones(size(X))]./vecnorm([planeNormal(1)/planeNormal(3)*ones(size(X));
            planeNormal(2)/planeNormal(3)*ones(size(X));
            ones(size(X))]);
        params.geomRange = [-10 10];

    case 'pyramid'
        baseSide = 5;
        height = 2;
        params.zFunc = @(X,Y) pyramidFunc(X,Y,baseSide,height);
        params.normalsFunc = @(X,Y) pyramidNormalsFunc(X,Y,baseSide,height);
        params.geomRange = [-5 5];
end

% Albedo
params.repCam = 255; % camera response
params.albedoImage = imread('Lena512.bmp'); % albedo image
% params.albedoImage = imread('car1.jpg'); % albedo image
params.albedoSize = size(params.albedoImage); % albedo size
w = params.albedoSize(2); h = params.albedoSize(1);
if h < w
    params.albedoImage = params.albedoImage(:,round((w-h)/2):round((w-h)/2+h-1),:);
    params.albedoSize = size(params.albedoImage); % albedo size
elseif h > w
    params.albedoImage = params.albedoImage(round((h-w)/2):round((h-w)/2+w-1),:,:);
    params.albedoSize = size(params.albedoImage); % albedo size
end
% params.geomRange = 2*[-params.R params.R]; % where you want to map the albedo on the geometry

% Albedo funtion depending on the renderType
switch params.renderType
    case '1'
        params.albedoImage = 1;
        params.albedoSize = size(params.albedoImage);
        params.albedoFunc = @(X,Y) ones(size(X));
    case 'gray'
        params.albedoImage = double(rgb2gray(params.albedoImage))/params.repCam;
        params.albedoSize = size(params.albedoImage);
        params.albedoFunc = @(X,Y) interp2(params.albedoImage,...
            mapToRange(X,params.geomRange,[1 params.albedoSize(2)]),...
            mapToRange(Y,-params.geomRange,[1 params.albedoSize(1)]),...
            'linear',0);
    case 'rgb'
        params.albedoImage = double(params.albedoImage)/params.repCam;
        params.albedoFunc = @(X,Y) albedoRGBFunc(X,Y,params.albedoImage, ...
            params.geomRange);
    case 'noise'
        params.albedoSize = [100 100];
        params.albedoImage = randn(params.albedoSize(1),params.albedoSize(2),3)+0.5;
        params.albedoFunc = @(X,Y) albedoRGBFunc(X,Y,params.albedoImage, ...
            params.geomRange);
    otherwise

end