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
2. Clone the repository:  git clone git@github.com:auralius/libkine.git 
3. Go to the directory of the cloned project, use git-lfs to get the binaries and the robot 3D models: git lfs checkout
4. Build the source using CMake
5. Create directory: mkdir build
6. Go inside build, type: cmake ..
7. Type: make
8. Run: ./libkine ../model/puma/puma.csv

---

Binaries and robot 3D models are stored with git-lfs (https://git-lfs.github.com).