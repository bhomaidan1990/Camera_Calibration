
# Triangulation

## Intro

- This function uses triangulation to reconstruct 3D points from their projections in two images and the corresponding camera matrices.  
  - Note that in this context, triangulation does not refer to subdivision into triangles (e.g., as in Delaunay).

- Instead, this function computes the intersection in space of rays defined by the camera centers and pairs of matching image projections. As these rays will probably be skew due to various sources of error, the image projections can optionally be corrected prior to triangulation so that they are consistent with the epipolar geometry. Correction might involve any of:

 1. global minimization of the error (optimal solution, involves 6th order polynomial)
 2. Sampson approximation of the error
 3. [Lindstrom's fast method](https://www.cc.gatech.edu/~turk/my_papers/memless_vis98.pdf)

- Option (1) is recommended, however, correction is not carried out by default.
- After any correction, the points are linearly triangulated with:

 1. DLT.
 2. 3D midpoint method.
 3. Via essential Matrix.
 4. Optimal Triangulation.

More details can be found in: 

[R. Hartley and P. Sturm "Triangulation", CVIU 68(2):146-157, 1997](http://users.cecs.anu.edu.au/~hartley/Papers/triangulation/triangulation.pdf)

Citation:
```
Manolis Lourakis (2020). Stereo triangulation (https://www.mathworks.com/matlabcentral/fileexchange/67383-stereo-triangulation), MATLAB Central File Exchange. Retrieved April 20, 2020.
```

[Manolis Lourakis (2020). Stereo triangulation](https://www.mathworks.com/matlabcentral/fileexchange/67383-stereo-triangulation)
