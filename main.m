%% Main script
close all
clear

% General parameters
display_ = 1;
dataPath = 'data/pyramid_rgb_1000/';
if exist(dataPath,'dir') % check if folder does NOT exist
    rmdir(dataPath,'s')
%     load([ dataPath 'data.mat' ]);
end

%% Set parameters
params = setParameters();

%% Display of the scene
if display_
    figure; hold on;
    switch params.geometryType
        case 'sphere'
            %x = -2:0.01:2;
            x = -40:0.1:40;
            y = x;
            [X,Y] = meshgrid(x,y);
            Z = params.zFunc(X,Y);
            isReal = imag(Z) < 1e-6;
            isAboveThrZ = Z > params.thrZ;
            Z(~(isReal&isAboveThrZ)) = params.thrZ;
            Z = real(Z);
            switch params.renderType
                case 'rgb'
                    surf(X,Y,Z,uint8(params.repCam*params.albedoFunc(X,Y)));
                otherwise
                    surf(X,Y,Z);
            end
        case 'multi-gaussian'
            x = -5:0.1:5;
            y = x;
            [X,Y] = meshgrid(x,y);
            surf(X,Y,params.zFunc(X,Y),uint8(params.repCam*params.albedoFunc(X,Y)));
%             surf(X,Y,params.zFunc(X,Y));
        otherwise
            x = -5:0.1:5;
            y = x;
            [X,Y] = meshgrid(x,y);
%             surf(X,Y,params.zFunc(X,Y),uint8(params.repCam*params.albedoFunc(X,Y)));
            surf(X,Y,params.zFunc(X,Y));
            normals = params.normalsFunc(X(:)',Y(:)');
            quiver3(X(:),Y(:),params.zFunc(X(:),Y(:)),normals(1,:)',normals(2,:)',normals(3,:)');
    end
    shading interp
    axis tight
    for i = 1:params.nCameras
        RCam = params.w2cPoses(:,1:3,i); 
        centerCam = squeeze(params.c2wPoses(:,4,i))';
        plotCamera('Orientation',RCam,'Location',centerCam,'Size',0.2);
    end
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal
%     xlim([-max(abs(X(isReal))) max(abs(X(isReal)))])
%     ylim([-max(abs(Y(isReal))) max(abs(Y(isReal)))])
%     zlim([0 centerCam(3)+5])
    rotate3d
    drawnow
end

%% Images rendering and depth maps
switch params.geometryType
    case 'sphere'
        [renderedImages,maskMaps,depthMaps,distMaps,normalMaps,albedoMaps] = ...
            renderSphere(params);
    otherwise
        [renderedImages,maskMaps,depthMaps,distMaps,normalMaps,albedoMaps] = ...
            render(params);
end

%% Post-processing depthMaps and normalMaps

% Depth maps
bounds = zeros(2,params.nCameras);
for ii = 1:params.nCameras
    bounds(:,ii) = [min(depthMaps(:,:,ii),[],'all'); ...
        max(depthMaps(:,:,ii),[],'all')]; %NEAR / FAR
end
depthMapsPlot = depthMaps/max(bounds(2,:));

% Normal maps
normalMapsPlot = normalMaps;
normalMapsPlot(:,:,2:3,:) = -normalMapsPlot(:,:,2:3,:);
normalMapsPlot = (normalMapsPlot+1)/2;

if display_
    if params.nCameras == 1
        indImage = 1;
    else
        indImage = 5;
    end
    figure; hold on;

    subplot(2,2,1);
    imshow(renderedImages(:,:,:,indImage,1));
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
imagesNoAlphaFolder = [ dataPath 'images_noalpha/' ];
depthFolder = [ dataPath 'depth/' ];
normalFolder = [ dataPath 'normal/' ];
albedoFolder = [ dataPath 'albedo/' ];
maskFolder = [ dataPath 'mask/' ];
mkdir(imagesFolder)
mkdir(imagesNoAlphaFolder)
mkdir(depthFolder)
mkdir(normalFolder)
mkdir(albedoFolder)
mkdir(maskFolder)

for ii = 1:params.nCameras
    for jj = 1:params.nLightSources
        imwrite(renderedImages(:,:,:,ii,jj), ...
            [ imagesFolder sprintf('%02d',ii) '_light_' ...
            sprintf('%02d',jj) '.png' ], ...
            'Alpha',double(maskMaps(:,:,ii)));
        imwrite(renderedImages(:,:,:,ii,jj), ...
            [ imagesNoAlphaFolder sprintf('%02d',ii) '_light_' ...
            sprintf('%02d',jj) '.png' ]);
    end
    exrwrite(depthMapsPlot(:,:,ii), ...
        [ depthFolder sprintf('%02d',ii) '.exr' ]);
    exrwrite(normalMapsPlot(:,:,:,ii), ...
        [ normalFolder sprintf('%02d',ii) '.exr' ]);
    imwrite(albedoMaps(:,:,:,ii), ...
        [ albedoFolder sprintf('%02d',ii) '.png' ], ...
        'Alpha',double(maskMaps(:,:,ii)));
    imwrite(double(maskMaps(:,:,ii)), ...
        [ maskFolder sprintf('%02d',ii) '.png' ]);
end

%% SAVE
exportMeshroom;

save([ dataPath 'data.mat' ],...
    'params','renderedImages','depthMaps','bounds','distMaps',...
    'normalMaps','albedoMaps','maskMaps','-v7.3')

%% Display

% load([ dataPath 'data.mat' ])
figure;
for ii = 1:params.nCameras
    subplot(3,3,ii);
    imshow(renderedImages(:,:,:,ii,1))
    title([ 'Image n°' num2str(ii) ])
end