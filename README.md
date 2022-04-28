# Lambertian Rendering v1

MATLAB implementation of a lambertian renderer without bounce from geometry, albedo, light and camera poses.

<p float="left">
  <img src="data/checkerboardExample/images/image_01.png" width="100" />
  <img src="data/checkerboardExample/images/image_12.png" width="100" /> 
  <img src="data/checkerboardExample/images/image_32.png" width="100" />
</p>

## Setup

Extract the ZIP file (or clone the git repository) somewhere you can easily reach it.

## Usage

### Setup parameters
Check out the function `setParameters.m` to change camera and scene parameters.

#### Rendering parameters
1. Basic rendering parameters are available as image size and color type (gray or RGB). 
2. Simple pinhole camera intrinsics are stored as a 3x3 matrix knowing the principal point and the focal length of the camera. 
3. Poses are stored as 3x4 matrices that represent **world-to-camera** transformation matrices `[ R t ]`.
4. Directional lighting is stored as a 3x1 vector.
5. **Perspective** or **orthographic** projection model are available.

#### Scene parameters
1. Scene **geometry** is defined as an analytical `z` function depending of the `x` and `y` components.
2. **Normals** are also defined analytically with the cross product between the `x` and `y` gradient components of the `z` function.
3. **Albedo** is stored as a function. Any grayscale or colored image can be used as the albedo.

### Running code
Once your parameters are set, run `main.m` script to get the rendered images of your scene.
