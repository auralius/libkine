libkine
---

A serial robot kinematics library.

Feature:
---

 * [x] DH Paramters
 * [x] Homogenous transformation matrix
 * [ ] Screw theory
 * [ ] Rigid body velocity / Jacobian
 * [x] 3D Visualization with using STL file

Dependencies:
---

1. VTK for visualization
2. Armadillo for matrix calculation

How to Install:
---

1. Make sure you have git-lfs installed
2. Clone the repository
3. Build the source using CMake
4. Create directory: mkdir build
5. Go inside build, type: cmake ..
6. Type: make
7. Run: ./libkine ../model/puma/puma.csv

Bnaries and models are stored using git-lfs (https://git-lfs.github.com).