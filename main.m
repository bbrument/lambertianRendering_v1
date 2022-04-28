%% Main script
close all
clear

% General parameters
display_ = 1;
dataPath = 'data/checkerboardExample/';
imagesFolder = [ dataPath 'images/' ];
mkdir(imagesFolder)

% Set parameters
params = setParameters();
nCams = params.nCameras;

% Display of the scene
if display_
    x = linspace(params.geomRange(1),params.geomRange(2));
    y = x;
    [X,Y] = meshgrid(x,y);
    figure; hold on;
    surf(X,Y,params.zFunc(X,Y),uint8(params.repCam*params.albedoFunc(X,Y)));
    shading interp
    axis tight
    for i = 1:nCams
        RCam = params.w2cPoses(:,1:3,i); 
        centerCam = -RCam'*squeeze(params.w2cPoses(:,4,i));
        plotCamera('Orientation',RCam,'Location',centerCam,'Size',0.2);
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal
    rotate3d
    drawnow
end

% Images rendering and depth maps
[renderedImages,depthMaps] = render(params);
% bounds = [min(depthMaps,[],'all'); max(depthMaps,[],'all')]; %NEAR / FAR

%% Save data and images
save([ dataPath 'data.mat' ],'params','renderedImages','depthMaps')
for ii = 1:nCams
    imwrite(uint8(renderedImages(:,:,:,ii)), ...
        [ imagesFolder 'image_' sprintf('%02d',ii) '.png' ]);
end