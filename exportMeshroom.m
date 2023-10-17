%% Export data to meshroom sfmdata
% close all
% clear

formatSpec = '%.12f';

% General parameters
% dataPath = 'data/front_plane_white_4000/';
% load([dataPath 'data.mat'])

%
if params.nChannels == 3
    images = dir([ dataPath 'images/*.png']);
else
    images = dir([ dataPath 'images_noalpha/*.png']);
end

% Number of views
numViews = params.nCameras;
imageSize = params.imageSize;
K = params.K;
w2cPoses = params.w2cPoses;
c2wPoses = params.c2wPoses;

% Inputs
intrinsicId = '1';
imageWidth = imageSize(2);
imageHeight = imageSize(1);
sensorWidth = 36;
sensorHeight = 36;

% Define float values as inputs
initialFocalLength = '0.001';
focalLength = K(1,1)*sensorWidth/imageWidth;
principalPoint = K(1:2,3)-[imageWidth;imageHeight]/2;

%% Create a structure for the JSON object
data = struct();

% Add version information
data.version = {'1', '2', '4'};

% Add features folders
data.featuresFolders = {};

% Add matches folders
data.matchesFolders = {};

% Create an array of views
for i = 1:numViews
    view = struct();
    view.viewId = num2str(i); % Generate viewId based on index
    view.poseId = view.viewId;
    view.frameId = num2str(i);
    view.intrinsicId = intrinsicId;
    view.path = fullfile(images(i).folder,images(i).name);
    view.width = num2str(imageWidth);
    view.height = num2str(imageHeight);
    view.metadata = '';
    data.views(i) = view;
end

% Create an array of intrinsics
intrinsic = struct();
intrinsic.intrinsicId = intrinsicId;
intrinsic.width = num2str(imageWidth);
intrinsic.height = num2str(imageHeight);
intrinsic.sensorWidth = num2str(sensorWidth);
intrinsic.sensorHeight = num2str(sensorHeight);
intrinsic.serialNumber = '0';
intrinsic.type = 'pinhole';
intrinsic.initializationMode = 'unknown';
intrinsic.initialFocalLength = num2str(initialFocalLength,formatSpec);
intrinsic.focalLength = num2str(focalLength,formatSpec);
intrinsic.pixelRatio = '1';
intrinsic.pixelRatioLocked = 'false';
intrinsic.principalPoint = num2str(principalPoint,formatSpec);
intrinsic.distortionInitializationMode = 'none';
intrinsic.distortionParams = '';
intrinsic.undistortionOffset = {'0', '0'};
intrinsic.undistortionParams = '';
intrinsic.locked = 'true';
data.intrinsics(1) = {intrinsic};

% Create an array of poses
for i = 1:numViews

    R = w2cPoses(:,1:3,i);
    center = squeeze(c2wPoses(:,4,i));

    poseObj.poseId = num2str(i); % Generate poseId based on index
    poseObj.pose.transform.rotation = num2str(R(:),formatSpec);
    poseObj.pose.transform.center = num2str(center(:),formatSpec);
    poseObj.pose.locked = '0';
    data.poses(i) = poseObj;
end

% Create fake structure
[X,Y] = meshgrid(-10:0.1:10,-10:0.1:10);
Z = params.zFunc(X(:),Y(:));
if strcmp(params.geometryType,'sphere')
    isReal = imag(Z) < 1e-3;
    isAboveThrZ = Z > params.thrZ;
    Z(~(isReal&isAboveThrZ)) = params.thrZ;
end
zMin = min(Z); zMax = max(Z);

for i = 1:2
    structObj.landmarkId = num2str(i);
    structObj.descType = 'dspsift';
    structObj.color = {'255', '255', '255'};
    if i == 1
        structObj.X = {'0', '0', num2str(zMin-0.5)};
    elseif i == 2
        structObj.X = {'0', '0', num2str(zMax+0.5)};
    end
    for ii = 1:numViews
        observation.observationId = num2str(ii);
        observation.featureId = '0';
        observation.x = {'1.0', '1.0'};
        observation.scale = '1.0';
        structObj.observations(ii) = observation;
    end
    data.structure(i) = structObj;
end

%% SAVE
fid = fopen([ dataPath 'sfm.json' ],'w');
jsonStr = jsonencode(data,PrettyPrint=true);
fprintf(fid,jsonStr);
fclose('all');