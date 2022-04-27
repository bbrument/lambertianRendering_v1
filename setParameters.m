%% Function to set parameters
function params = setParameters()

%% Camera parameters
params.factor = 2;
params.imageSize = [1000 1000]/params.factor;
params.nChannels = 1;
params.renderType = 'bw'; % '1', 'bw' or 'rgb'
if strcmp(params.renderType,'rgb')
    params.imageSize = [params.imageSize 3];
    params.nChannels = 3;
end

% Calibration matrix
px = 500/params.factor;
py = 500/params.factor;
f = 2000/params.factor;
params.K = [f 0 px ; 0 f py ; 0 0 1];

% Camera poses
cameraGrid = [6 6];
cameraRange = 3;
params.w2cPoses = multiView(cameraGrid,cameraRange);
params.nCameras = size(params.w2cPoses,3);

% Orthographic or perspective camera?
params.cameraType = 'ortho'; % 'persp' or 'ortho'

% World-to-camera matrix (w2c) -> Camera-to-world matrix (c2w)
params.c2wPoses = zeros(size(params.w2cPoses));
params.c2wPoses(:,1:3,:) = pagetranspose(params.w2cPoses(:,1:3,:));
params.c2wPoses(:,4,:) = -pagemtimes(params.w2cPoses(:,1:3,:),'transpose',...
    params.w2cPoses(:,4,:),'None');

%% Light source parameters
params.lightIntensity = 1;
params.lightSource = [0;0;1];

%% Scene parameters

% Geometry
mu = [0.2 0.4];
sigma = [0.7 0.6];
params.zFunc = @(X,Y) 1/(sigma(1)*sigma(2)*sqrt(2*pi))...
    .*exp(-1/2*((X-mu(1)).^2/sigma(1)^2 + (Y-mu(2)).^2/sigma(2)^2));

% Normals
params.normalsFunc = @(X,Y) [-(2^(1/2).*exp(-(mu(1) - X).^2/(2*sigma(1)^2) - (mu(2)-Y).^2/(2*sigma(2)^2)).*(2*mu(1) - 2*X))/(4*pi^(1/2)*sigma(1)^4);
        -(2^(1/2).*exp(- (mu(1) - X).^2/(2*sigma(1)^2) - (mu(2) - Y).^2/(2*sigma(2)^2)).*(2*mu(2) - 2*Y))/(4*pi^(1/2)*sigma(1)^2*sigma(2)^2);
        ones(1,size(X,2))];

% Albedo
params.repCam = 255;
params.albedoImage = imread('damier.jpg');
params.albedoSize = size(params.albedoImage);
params.geomRange = [-5 5];

switch params.renderType
    case '1'
        params.albedoImage = 1;
        params.albedoSize = size(params.albedoImage);
        params.albedoFunc = @(X,Y) ones(size(X));
    case 'bw'
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



