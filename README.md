# LiODOM - Adaptive Local Mapping for Robust LiDAR-Only Odometry

LiODOM is an open source C++ library for LiDAR-Only pose estimation and map building. It is based on minimizing a loss function derived from a set of weighted edge-to-line correspondences with a local map. The (unoptimized) global map is represented by a fast and efficient hash-based grid structure that speeds up searches and updates. The local map is obtained from the global map according to the current position of the vehicle.

Note that LiODOM is research code. The authors are not responsible for any errors it may contain. 

**USE IT AT YOUR OWN RISK!**

# Conditions of use

LiODOM is distributed under the terms of the [GPL3 License](http://github.com/emiliofidalgo/liodom/blob/master/LICENSE).

# Related publication

The details of the algorithm are explained in the following [publication](https://www.sciencedirect.com/science/article/pii/S0921889022001324):

**LiODOM: Adaptive Local Mapping for Robust LiDAR-Only Odometry**<br/>
Emilio Garcia-Fidalgo, Joan P. Company-Corcoles, Francisco Bonnin-Pascual and Alberto Ortiz<br/>
Robotics and Autonomous Systems 156, 104226, 2022<br/>

If you use this code, please cite as:
```
@article{Garcia-Fidalgo2022,
	title = {{LiODOM: Adaptive Local Mapping for Robust LiDAR-Only Odometry}},
	journal = {Robotics and Autonomous Systems},
	volume = {156},
	pages = {104226},
	year = {2022},
	issn = {0921-8890},
	doi = {https://doi.org/10.1016/j.robot.2022.104226},
	author = {Emilio Garcia-Fidalgo and Joan P. Company-Corcoles and Francisco Bonnin-Pascual and Alberto Ortiz},
}
```

# Installation

## Prerequisites
- Tested on [Ubuntu 64-bit 20.04](http://ubuntu.com/download/desktop)
- Tested on [ROS Noetic](http://wiki.ros.org/ROS/Installation)
- [Ceres](http://ceres-solver.org/installation.html)
- [PCL](http://pointclouds.org/)

## Build
```
  cd ~/your_workspace/src
  git clone https://github.com/emiliofidalgo/liodom.git
  cd ..
  catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Usage

For an example of use, see the launch file `launch/liodom.launch`.

Depending on the computer on which LiODOM is running, there are three critical parameters that can affect its performance:
- `scan_regions`: The number of regions in which each horizontal scan is divided.
- `edges_per_region`: The number of edges to detect on each region.
- `prev_frames`: The number of previous frames to be maintained in the local map.

Reducing the values of these parameters may speed up LiODOM, sacrificing accuracy. Adjust them to your needs!

# Contact

If you have problems or questions using this code, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/liodom/issues) and [contributions](http://github.com/emiliofidalgo/liodom/pulls) are totally welcome.

# Acknowledgements
Thanks to the authors of [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [F-LOAM](https://github.com/wh200720041/floam) for publishing their codes. Some parts of this library are inspired from them.
