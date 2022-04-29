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
[renderedImages,depthMaps,distMaps,normalMaps,albedoMaps] = render(params);

%% Post-processing depthMaps and normalMaps

% Depth maps
bounds = zeros(2,nCams);
for ii = 1:nCams
    bounds(:,ii) = [min(depthMaps(:,:,ii),[],'all'); ...
        max(depthMaps(:,:,ii),[],'all')]; %NEAR / FAR
end
depthMapsPlot = depthMaps/max(bounds(2,:));

% Normal maps
normalMapsPlot = (normalMaps+1)/2;

if display_
    figure; hold on;

    subplot(2,2,1);
    imshow(renderedImages(:,:,:,1));
    title('Rendered image')

    subplot(2,2,2);
    imshow(depthMapsPlot(:,:,1));
    title('Depth map')

    subplot(2,2,3);
    imshow(normalMapsPlot(:,:,:,1));
    title('Normal map')

    subplot(2,2,4);
    imshow(albedoMaps(:,:,:,1));
    title('Albedo map')
end


%% Save data and images
save([ dataPath 'data.mat' ],...
    'params','renderedImages','depthMaps','bounds','distMaps',...
    'normalMaps','albedoMaps')
for ii = 1:nCams
    imwrite(renderedImages(:,:,:,ii), ...
        [ imagesFolder 'image_' sprintf('%02d',ii) '.png' ]);
end