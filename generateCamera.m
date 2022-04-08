function RCam = generateCamera(cameraPos,pointToLookAt)

% Camera rotation expressed in the world coordinate system pointing down
R = [  1       0                 0
    0       cos(pi)  -sin(pi)
    0       sin(pi)   cos(pi)  ];

% Make the camera point to pointToLookAt
theta = atan((cameraPos(2)-pointToLookAt(2))./...
    (cameraPos(3)-pointToLookAt(3))); % rot around x-axis
phi = atan((cameraPos(1)-pointToLookAt(1))./...
    (cameraPos(3)-pointToLookAt(3))); % rot around y-axis

% Rotation matrix
rotX = [  1       0                 0;
        0       cos(theta)  -sin(theta);
        0       sin(theta)   cos(theta)  ];
rotY = [  cos(phi)       0     -sin(phi);
        0              1   0;
        sin(phi)       0   cos(phi)  ];
RCam = R*rotX*rotY;