%% Main script
close all
clear

% General parameters
display_ = 1;
dataPath = 'data/checkerboardExample/';

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
[renderedImages,depthMaps,distMaps,normalMaps,albedoMaps,pointMaps] = render(params);

%% Post-processing depthMaps and normalMaps

% Depth maps
bounds = zeros(2,nCams);
for ii = 1:nCams
    bounds(:,ii) = [min(depthMaps(:,:,ii),[],'all'); ...
        max(depthMaps(:,:,ii),[],'all')]; %NEAR / FAR
end
depthMapsPlot = depthMaps/max(bounds(2,:));

% Normal maps
normalMapsPlot = normalMaps;
normalMapsPlot(:,:,2:3,:) = -normalMapsPlot(:,:,2:3,:);
normalMapsPlot = (normalMapsPlot+1)/2;

% if display_
%     figure; hold on;
%     set(gca, 'YDir','reverse')
%     axis equal
%     indImage = 5;
%     step = 20;
% 
%     projMat = params.K * params.w2cPoses(:,:,indImage);
%     for ii = 1:step:500
%         for jj = 1:step:500
%             
%             point = projMat*[ squeeze(pointMaps(ii,jj,:,indImage)); 1 ];
%             normalPoint = projMat*[ squeeze(pointMaps(ii,jj,:,indImage) + ...
%                 normalMaps(ii,jj,:,indImage)); 1 ];
% 
%             point = point/point(3);
%             normalPoint = normalPoint/normalPoint(3);
%             normalProj = normalPoint - point;
% 
%             plot(point(1),point(2),'mx');
%             quiver(point(1),point(2),normalProj(1),normalProj(2),0.1,'b');
%         end
%     end
% 
%     
% end

if display_
    indImage = 5;
    figure; hold on;

    subplot(2,2,1);
    imshow(renderedImages(:,:,:,indImage));
    title('Rendered image')

    subplot(2,2,2);
    imshow(depthMapsPlot(:,:,indImage));
    title('Depth map')

    subplot(2,2,3);
    imshow(normalMapsPlot(:,:,:,indImage));
    title('Normal map')

    subplot(2,2,4);
    imshow(albedoMaps(:,:,:,indImage));
    title('Albedo map')
end


%% Save data and images
imagesFolder = [ dataPath 'images/' ];
depthFolder = [ dataPath 'depth/' ];
normalFolder = [ dataPath 'normal/' ];
albedoFolder = [ dataPath 'albedo/' ];
mkdir(imagesFolder)
mkdir(depthFolder)
mkdir(normalFolder)
mkdir(albedoFolder)

for ii = 1:nCams
    imwrite(renderedImages(:,:,:,ii), ...
        [ imagesFolder sprintf('%02d',ii) '.png' ]);
    imwrite(depthMapsPlot(:,:,ii), ...
        [ depthFolder sprintf('%02d',ii) '.png' ]);
    imwrite(normalMapsPlot(:,:,:,ii), ...
        [ normalFolder sprintf('%02d',ii) '.png' ]);
    imwrite(albedoMaps(:,:,:,ii), ...
        [ albedoFolder sprintf('%02d',ii) '.png' ]);
end

save([ dataPath 'data.mat' ],...
    'params','renderedImages','depthMaps','bounds','distMaps',...
    'normalMaps','albedoMaps')