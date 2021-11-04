# LiODOM - Adaptive Local Mapping for Robust LiDAR-Only Odometry

LiODOM is an open source C++ library for LiDAR-Only pose estimation and map building. It is based on minimizing a loss function derived from a set of weighted edge-to-line correspondences with a local map. The (unoptimized) global map is represented by a fast and efficient hash-based grid structure that speeds up searches and updates. The local map is obtained from the global map according to the current position of the vehicle.

Note that LiODOM is research code. The authors are not responsible for any errors it may contain. **Use it at your own risk!**

# Conditions of use

LiODOM is distributed under the terms of the [GPL3 License](http://github.com/emiliofidalgo/liodom/blob/master/LICENSE).

# Related publication

The details of the algorithm are explained in the following publication:

**LiODOM: Adaptive Local Mapping for Robust LiDAR-Only Odometry**<br/>
Emilio Garcia-Fidalgo, Joan P. Company-Corcoles, Francisco Bonnin-Pascual and Alberto Ortiz<br/>
Submitted to a journal for possible publication<br/>

# Installation

## Prerequisites
- Tested on [Ubuntu 64-bit 20.04](http://ubuntu.com/download/desktop).
- Tested on [ROS Noetic](http://wiki.ros.org/ROS/Installation).
- [Ceres](http://ceres-solver.org/installation.html).
- [PCL](http://pointclouds.org/).

## Build
```
  cd ~/your_workspace/src
  git clone https://github.com/emiliofidalgo/liodom.git
  cd ..
  catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Usage

For an example of use, see the launch file `launch/liodom.launch`.

# Contact

If you have problems or questions using this code, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/liodom/issues) and [contributions](http://github.com/emiliofidalgo/liodom/pulls) are totally welcome.