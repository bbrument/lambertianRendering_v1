# Lambertian Rendering v1

MATLAB implementation of a lambertian renderer without bounce from geometry, albedo, light and camera poses.

<p float="left">
  <img src="data/checkerboardExample/images/01.png" width="250" />
  <img src="data/checkerboardExample/images/05.png" width="250" /> 
  <img src="data/checkerboardExample/images/09.png" width="250" />
</p>

## Setup

Extract the ZIP file (or clone the git repository) somewhere you can easily reach it.

## Usage

### Setup parameters
Check out the function `setParameters.m` to change the camera and scene parameters.

#### Rendering parameters
1. Basic rendering parameters are available such as image size and color type (gray or RGB). 
2. Camera intrinsics are stored in a 3x3 calibration matrix that includes principal point coordinates and focal length. 
3. Poses are stored in 3x4 matrices that represent **world-to-camera** transformation matrices `[ R t ]`.
4. Directional lighting is stored in a vector-3.
5. **Perspective** or **orthographic** projection models are available.

#### Scene parameters
1. The scene **geometry** is a surface defined by a bivariate function `z(x,y)`.
4. Its **normals** are the cross product between the `x` and `y` gradient of the `z` function.
5. The **albedo** is a piecewise linear function of (x,y) whose coefficients are defined by an image. Any grayscale or colored image can be used to define it (see `setParameters.m`).

### Running code
Once your parameters are set, run `main.m` script to get the rendered images of your scene.
Corresponding depth, normal and albedo maps are computed and stored in folders defined in `main.m`.
