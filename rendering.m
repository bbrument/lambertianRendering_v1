%% Main script
close all
clear

% Options
display_ = 1;
dataPath = 'data/bivar_gaussian_200_with_albedo/';
mkdir(dataPath);

% Rendering options
repCam = 255;
scale = 5;
imageSize = 1000/scale;

% Calibration matrix
px = 500/scale;
py = 500/scale;
f = 1500/scale;
K = [f 0 px ; 0 f py ; 0 0 1];

% Geometry
%sigma = 0.5;
%zFunc = @(X,Y) 1/(sigma*sqrt(2*pi)).*exp(-(X.^2/(2*sigma^2)));
geomRange = [-5 5];
mu = [0.2 0.4];
sigma = [0.7 0.6];
zFunc = @(X,Y) 1/(sigma(1)*sigma(2)*sqrt(2*pi)).*exp(-1/2*((X-mu(1)).^2/sigma(1)^2 + (Y-mu(2)).^2/sigma(2)^2));

% Albedo
albedoSize = 500;
albedoRange = [1 albedoSize];
im = double(rgb2gray(imread('peppers.png')))/repCam; % with albedo
%im = 1; % without albedo
im = imresize(im,[albedoSize albedoSize]);
albedoBinFunc = @(X,Y) interp2(im,mapToRange(X,geomRange,albedoRange),mapToRange(Y,geomRange,albedoRange),...
    'cubic',0);

if display_
    x = linspace(geomRange(1),geomRange(2));
    y = x;
    [X,Y] = meshgrid(x,y);
    figure; hold on;
    surf(X,Y,zFunc(X,Y),uint8(repCam*albedoBinFunc(X,Y)));
    shading interp
    axis tight
    axis equal
end

% Normals
normalsFunc = @(X,Y) [-(2^(1/2).*exp(-(mu(1) - X).^2/(2*sigma(1)^2) - (mu(2)-Y).^2/(2*sigma(2)^2)).*(2*mu(1) - 2*X))/(4*pi^(1/2)*sigma(1)^4);
    -(2^(1/2).*exp(- (mu(1) - X).^2/(2*sigma(1)^2) - (mu(2) - Y).^2/(2*sigma(2)^2)).*(2*mu(2) - 2*Y))/(4*pi^(1/2)*sigma(1)^2*sigma(2)^2);
    ones(1,size(X,2))];

% Multi view & multi light
intLight = 1;
nCameras = 9;
cameraRange = 2;
nLights = 9;
lightRange = 2;
[RCamTab,centerCamTab,lightSourceTab] = multiViewLight(nCameras,cameraRange,nLights,lightRange);
save([ dataPath 'data_gt.mat' ],'RCamTab','centerCamTab','lightSourceTab');

% Display camera
if display_
    for i = 1:nCameras
        RCam = RCamTab(:,:,i); centerCam = centerCamTab(:,i);
        plotCamera('AbsolutePose',rigid3d(RCam,centerCam'),'Size',0.1);
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    %     pause
end

% Folder creation
for i = 1:nCameras
    mkdir([ dataPath 'view_' sprintf('%02d/',i) ]);
    for j = 1:nLights
        mkdir([ dataPath 'light_' sprintf('%02d/',j) ]);
    end
end

% Rendering
for i = 1:nCameras
    disp([ 'View : ' int2str(i) ]);
    RCam = RCamTab(:,:,i); tCam = -RCam*centerCamTab(:,i);
    renderedImages = render(zFunc,normalsFunc,RCam,tCam,lightSourceTab,albedoBinFunc,K,repCam,intLight,imageSize);
    for j = 1:nLights
        imwrite(renderedImages(:,:,:,j),[ dataPath 'light_' sprintf('%02d/',j) ...
            'image_' sprintf('%02d',i) '.png' ]);
        imwrite(renderedImages(:,:,:,j),[ dataPath 'view_' sprintf('%02d/',i) ...
            'image_' sprintf('%02d',j) '.png' ]);s
    end

    if display_
        if i == 1
            figure;
        end
        subplot(3,3,i);
        imshow(renderedImages(:,:,:,5));
        title([ 'View ' sprintf('%02d',i) ]);  
    end
end