%% Main script
close all
clear

% Options
display_ = 1;
dataPath = 'data/oneLightSourceAlbRGB/';
mkdir(dataPath);

% Rendering options
repCam = 255;
factor = 2;
imageSize = 1000/factor;

% Calibration matrix
px = 500/factor;
py = 500/factor;
f = 2000/factor;
K = [f 0 px ; 0 f py ; 0 0 1];

% Geometry
%sigma = 0.5;
%zFunc = @(X,Y) 1/(sigma*sqrt(2*pi)).*exp(-(X.^2/(2*sigma^2)));
geomRange = [-5 5];
mu = [0.2 0.4];
sigma = [0.7 0.6];
zFunc = @(X,Y) 1/(sigma(1)*sigma(2)*sqrt(2*pi))...
    .*exp(-1/2*((X-mu(1)).^2/sigma(1)^2 + (Y-mu(2)).^2/sigma(2)^2));

% Albedo
albedoSize = 600;
albedoRange = [1 albedoSize];
im = double(rgb2gray(imread('peppers.png')))/repCam; % with BW albedo
%im = 1; % without albedo
im = imresize(im,[albedoSize albedoSize]);
albedoBinFunc = @(X,Y) interp2(im,mapToRange(X,geomRange,albedoRange),...
    mapToRange(Y,geomRange,albedoRange),...
    'cubic',0);

im = double(imread('peppers.png'))/repCam; % with RGB albedo
im = imresize(im,[albedoSize albedoSize]);
albedoRGBFunc = @(X,Y) [ interp2(im(:,:,1),mapToRange(X,geomRange,albedoRange),...
    mapToRange(Y,geomRange,albedoRange),'cubic',0);
    interp2(im(:,:,2),mapToRange(X,geomRange,albedoRange),...
    mapToRange(Y,geomRange,albedoRange),'cubic',0);
    interp2(im(:,:,3),mapToRange(X,geomRange,albedoRange),...
    mapToRange(Y,geomRange,albedoRange),'cubic',0) ];

if display_
    x = linspace(geomRange(1),geomRange(2));
    y = x;
    [X,Y] = meshgrid(x,y);
    figure; hold on;
    surf(X,Y,zFunc(X,Y),uint8(repCam*albedoBinFunc(X,Y)));
    shading interp
    axis tight
    axis equal
    rotate3d
end

% Normals
normalsFunc = @(X,Y) [-(2^(1/2).*exp(-(mu(1) - X).^2/(2*sigma(1)^2) - (mu(2)-Y).^2/(2*sigma(2)^2)).*(2*mu(1) - 2*X))/(4*pi^(1/2)*sigma(1)^4);
    -(2^(1/2).*exp(- (mu(1) - X).^2/(2*sigma(1)^2) - (mu(2) - Y).^2/(2*sigma(2)^2)).*(2*mu(2) - 2*Y))/(4*pi^(1/2)*sigma(1)^2*sigma(2)^2);
    ones(1,size(X,2))];

% Multi view & multi light
intLight = 1;
cameraGrid = [6 6];
nCameras = prod(cameraGrid);
cameraRange = 3;
lightGrid = [1 1];
nLights = prod(lightGrid);
lightRange = 2;
[w2cPoses,lightSourceTab] = multiViewLight(cameraGrid,...
    cameraRange,lightGrid,lightRange);
%save([ dataPath 'data_gt.mat' ],'RCamTab','centerCamTab','lightSourceTab');

% Display camera
if display_
    for i = 1:nCameras
        RCam = w2cPoses(:,1:3,i); centerCam = squeeze(w2cPoses(:,4,i));
        plotCamera('Orientation',RCam,'Location',centerCam,'Size',0.1);
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    rotate3d
    drawnow
end

% w2c -> c2w poses
c2wPoses = zeros(size(w2cPoses));
%c2wPoses(:,1:3,:) = pagetranspose(w2cPoses(:,1:3,:));
c2wPoses(:,1:3,:) = w2cPoses(:,1:3,:);
c2wPoses(:,4,:) = -pagemtimes(w2cPoses(:,1:3,:),'None',...
    w2cPoses(:,4,:),'None');

% Folder creation
if nLights > 1
    for i = 1:nCameras
        mkdir([ dataPath 'view_' sprintf('%02d/',i) '/images/' ]);
    end
end

if nCameras > 1
    for j = 1:nLights
        mkdir([ dataPath 'light_' sprintf('%02d/',j) '/images/' ]);
        poses = c2wPoses;
        intrinsics = K;
        save([ dataPath 'light_' sprintf('%02d/',j) '/data.mat' ],...
            'poses','intrinsics')
    end
end

% Rendering
bds = zeros(2,nCameras);
parfor i = 1:nCameras
    disp([ 'View : ' int2str(i) ]);

    % Rendering BW images
%     [renderedImages,depthMap] = render(zFunc,normalsFunc,c2wPoses(:,:,i),...
%         lightSourceTab,albedoBinFunc,K,repCam,intLight,imageSize);

    % Rendering RGB images
    [renderedImages,depthMap] = render(zFunc,normalsFunc,c2wPoses(:,:,i),...
        lightSourceTab,albedoRGBFunc,K,repCam,intLight,imageSize);
    bds(:,i) = [max(depthMap,[],'all'); min(depthMap,[],'all')];

    % Save images
    for j = 1:nLights
        if nCameras > 1
            imwrite(renderedImages(:,:,:,j),[ dataPath 'light_' sprintf('%02d/',j) ...
                '/images/image_' sprintf('%02d',i) '.png' ]);
        end
        if nLights > 1
            imwrite(renderedImages(:,:,:,j),[ dataPath 'view_' sprintf('%02d/',i) ...
                '/images/image_' sprintf('%02d',j) '.png' ]);
        end
    end
end
save([ dataPath 'light_01/data.mat' ],...
    'bds','-append')