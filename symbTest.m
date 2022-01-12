clear
syms x y z mu1 mu2 sigma1 sigma2 pi real

% 3D points
z = 1/(sigma1*sigma2*sqrt(2*pi))*exp(-1/2*((x-mu1)^2/sigma1^2 + (y-mu2)^2/sigma2^2));
X = [x;y;z];

% Normal
normals = cross(diff(X,x), diff(X,y))


