function albedo = albedoRGBFunc(X,Y,albedoImage,geomRange)

albedoSize = size(albedoImage);

X_pixel = mapToRange(X,geomRange,[1 albedoSize(2)]);
Y_pixel = mapToRange(Y,-geomRange,[1 albedoSize(1)]);

% albedoR = interp2(albedoImage(:,:,1),X_pixel,Y_pixel,'linear',0);
% albedoG = interp2(albedoImage(:,:,2),X_pixel,Y_pixel,'linear',0);
% albedoB = interp2(albedoImage(:,:,3),X_pixel,Y_pixel,'linear',0);

albedoR = interp2(albedoImage(:,:,1),X_pixel,Y_pixel,'cubic',0);
albedoG = interp2(albedoImage(:,:,2),X_pixel,Y_pixel,'cubic',0);
albedoB = interp2(albedoImage(:,:,3),X_pixel,Y_pixel,'cubic',0);

% albedoR = interp2(albedoImage(:,:,1),X_pixel,Y_pixel,'nearest',0);
% albedoG = interp2(albedoImage(:,:,2),X_pixel,Y_pixel,'nearest',0);
% albedoB = interp2(albedoImage(:,:,3),X_pixel,Y_pixel,'nearest',0);

albedo = cat(3,albedoR,albedoG,albedoB);