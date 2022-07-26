%% Function to set parameters
function params = setParameters()

% Camera parameters
params.factor = 5; % downscale factor
params.imageSize = [1000 1000]/params.factor; % rendered image size
params.nChannels = 1; % number of channels of the rendered image
params.renderType = 'gray'; % '1' for an homogeneous albedo, 'gray' for gray-scaled one or 'rgb' for an colored one
if strcmp(params.renderType,'rgb') % if renderType is 'rgb', update imageSize and nChannels
    params.imageSize = [params.imageSize 3]; 
    params.nChannels = 3;
end

% Camera intrinsics
px = 500/params.factor; % principal point
py = 500/params.factor;
f = 2000/params.factor; % focal length
params.K = [f 0 px ; 0 f py ; 0 0 1];

% Camera poses (generated on a grid pointing at the center of the scene)
cameraGrid = [3 3]; % number of cameras along x and y
cameraRange = 3; % range along both axis
cameraHeight = 95;
pointLookAt = [0,0,90];
params.w2cPoses = multiView(cameraGrid,cameraRange,...
    cameraHeight,pointLookAt); % cameras poses (world-to-camera matrices)
params.nCameras = size(params.w2cPoses,3); % number of cameras

% Orthographic or perspective camera?
params.cameraType = 'ortho'; % 'persp' or 'ortho'
params.orthoScale = 1;%200/params.factor;

% World-to-camera matrix (w2c) -> Camera-to-world matrix (c2w)
params.c2wPoses = zeros(size(params.w2cPoses));
params.c2wPoses(:,1:3,:) = pagetranspose(params.w2cPoses(:,1:3,:));
params.c2wPoses(:,4,:) = -pagemtimes(params.w2cPoses(:,1:3,:),'transpose',...
    params.w2cPoses(:,4,:),'None');

% Light source parameters
params.lightIntensity = 1; % light intensity
params.lightSource = [0;0;1]; % directional light source (default: light comes from above)

% Scene parameters

% Geometry (here is a bi-variate gaussian)
% mu = [0.2 0.4];
% sigma = [0.7 0.6];
% params.zFunc = @(X,Y) 1/(sigma(1)*sigma(2)*sqrt(2*pi))...
%     .*exp(-1/2*((X-mu(1)).^2/sigma(1)^2 + (Y-mu(2)).^2/sigma(2)^2));

% Geometry (sphere)
params.R = 90;
params.c = [0 0 0];
params.thrZ = params.R*sqrt(1-0.7^2);
params.zFunc = @(X,Y) sqrt(params.R^2 - (X-params.c(1)).^2 - (Y-params.c(2)).^2) + params.c(3);

% Normals 
% (calculated analytically : cross product of the x and y gradient components of the zFunc)
% params.normalsFunc = @(X,Y) [-(2^(1/2).*exp(-(mu(1) - X).^2/(2*sigma(1)^2) - (mu(2)-Y).^2/(2*sigma(2)^2)).*(2*mu(1) - 2*X))/(4*pi^(1/2)*sigma(1)^4);
%         -(2^(1/2).*exp(- (mu(1) - X).^2/(2*sigma(1)^2) - (mu(2) - Y).^2/(2*sigma(2)^2)).*(2*mu(2) - 2*Y))/(4*pi^(1/2)*sigma(1)^2*sigma(2)^2);
%         ones(1,size(X,2))];

params.normalsFunc = ...
    @(X,Y) [(X-params.c(1))./sqrt(params.R^2 - (X-params.c(1)).^2 - (Y-params.c(2)).^2);
            (Y-params.c(2))./sqrt(params.R^2 - (X-params.c(1)).^2 - (Y-params.c(2)).^2);
            ones(1,size(X,2))];

% Albedo
params.repCam = 255; % camera response
params.albedoImage = imread('checkerboard.png'); % albedo image
params.albedoSize = size(params.albedoImage); % albedo size
params.geomRange = [-90 90]; % where you want to map the albedo on the geometry

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
            mapToRange(Y,params.geomRange,[1 params.albedoSize(1)]),...
            'cubic',0);
    case 'rgb'
        params.albedoImage = double(params.albedoImage)/params.repCam;
        params.albedoFunc = @(X,Y) [ interp2(params.albedoImage(:,:,1),...
            mapToRange(X,params.geomRange,[1 params.albedoSize(2)]),...
            mapToRange(Y,params.geomRange,[1 params.albedoSize(1)]),...
            'cubic',0);
            interp2(params.albedoImage(:,:,2),...
            mapToRange(X,params.geomRange,[1 params.albedoSize(2)]),...
            mapToRange(Y,params.geomRange,[1 params.albedoSize(1)]),...
            'cubic',0);
            interp2(params.albedoImage(:,:,3),...
            mapToRange(X,params.geomRange,[1 params.albedoSize(2)]),...
            mapToRange(Y,params.geomRange,[1 params.albedoSize(1)]),...
            'cubic',0) ];
    otherwise
        
end