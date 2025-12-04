## List of Known Dependencies

### ORB-SLAM2 ROS node version 1.0

This document summarizes all third‑party code and libraries that are included in, or linked by, the ORB-SLAM2 ROS node but are **not** authored by the ORB-SLAM2 authors.  
It is intended to give you a clear overview of what external components are used, where they come from, and under which licenses they are distributed.

---

##### Code in `orb_slam2/src` and `orb_slam2/include` folders

- **`ORBextractor.cc`**  
  This file contains a modified version of the `orb.cpp` implementation from the OpenCV library.  
  The original OpenCV implementation is licensed under the **BSD** license.  
  ORB-SLAM2 adapts this code to better suit its feature extraction pipeline (e.g., parameterization, performance tuning), while preserving the original algorithmic basis.

- **`PnPsolver.h`, `PnPsolver.cc`**  
  These files are based on the `epnp.h` and `epnp.cc` implementation by Vincent Lepetit, which provide the Efficient Perspective-n-Point (EPnP) algorithm.  
  The same code can also be found in widely used BSD‑licensed computer vision libraries such as:
  - [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp)
  - [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp)  
  The original EPnP implementation is released under the **FreeBSD** license.  
  ORB-SLAM2 integrates and slightly modifies this solver to compute camera pose from 2D–3D correspondences.

- **Function `ORBmatcher::DescriptorDistance` in `ORBmatcher.cc`**  
  This function implements an efficient population‑count / Hamming distance bit‑trick.  
  The code is taken from the well‑known collection at  
  `http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel`.  
  That source is explicitly stated to be in the **public domain**, meaning it can be freely used, modified and redistributed.

---

##### Code in `Thirdparty` folder

- **All code in `Thirdparty/DBoW2`**  
  This folder contains a modified version of the [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) libraries by Dorian Gálvez-López.  
  These libraries implement a bag‑of‑words place recognition system used by ORB-SLAM2 for loop detection and relocalization.  
  All included files in this directory are distributed under the **BSD** license (as in the original projects).

- **All code in `Thirdparty/g2o`**  
  This folder contains a modified version of the [g2o](https://github.com/RainerKuemmerle/g2o) (General Graph Optimization) library.  
  g2o is used by ORB-SLAM2 for non‑linear optimization tasks such as bundle adjustment and pose graph optimization.  
  All files included from g2o are also distributed under the **BSD** license.

---

##### Library dependencies (linked at build/run time)

- **OpenCV**  
  OpenCV is a widely used computer vision library that ORB-SLAM2 uses for image handling, feature extraction, matrix operations and other low‑level vision utilities.  
  OpenCV is licensed under the **BSD** license, which permits commercial and closed‑source usage under certain conditions.

- **Eigen3**  
  Eigen3 is a high‑performance C++ template library for linear algebra (vectors, matrices, solvers, etc.) used extensively throughout ORB-SLAM2’s optimization and geometry code.  
  - For versions **≥ 3.1.1**, Eigen is licensed under **MPL 2.0**.  
  - For earlier versions, it is licensed under **LGPLv3**.

- **ROS**  
  The ORB-SLAM2 ROS node depends on various standard ROS packages for message passing, tf transforms and image transport.  
  The core ROS packages we depend on are:
  - `roscpp`
  - `tf`
  - `sensor_msgs`
  - `image_transport`
  - `cv_bridge`  
  These packages are all licensed under the **BSD** license.  
  This combination allows the ORB-SLAM2 ROS node to be used in a wide range of projects, including many commercial and research applications, while still respecting the underlying open‑source licenses.
