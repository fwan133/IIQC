# IIQC

### V1.0 Introduction

This project aims to propose a systematic image quality check framework for rapidly assessing the quality of UAV-captured images in alignment with inspection requirements and providing immediate feedback of unqualified UAV image for prompt image recollection when necessary. A set of image quality metrics aspects is proposed for UAV-enabled bridge inspection as well as a coarse-to-fine image pose estimation method to accurately obtain the relative pose between the captured images and the inspected bridge. Moreover, a compact and memory-efficient 3D representation model has been designed to serve as a medium for visualising the outcomes of the image quality assessment. The performance of the proposed framework was thoroughly validated through extensive experiments in both simulation and real-world environments, examining the precision of the coarse-to-fine image pose estimation, the effectiveness of pixel-level image quality metrics, as well as the performance of the enriched bridge representation model. A [validation video](https://youtu.be/qsGtplIuAD0) of the proposed method is also available. 

### Related publication



## 1. License



## 2. Prerequisites

The work has been tested under **Ubuntu 20.04**, but it should be easy to compile in other platforms. 

### C++11

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0 and 4.4.0**.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### g2o and Ceres
We use [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. 

### octomap
We use [OctoMap](https://github.com/OctoMap/octomap) to iteratively segment the 3D space into smaller cubes for rapid updates of the image quality check results.

### PCL
We use modified [PCL](https://github.com/PointCloudLibrary/pcl) functions to address bridge specific point clould registration problem.

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

### ROS 
The work has been tested with ROS Melodic under Ubuntu 20.04.

### Python
Used as additional tools under *scripts* folder for semantic point cloud conversion, loading .xml file and plotting results of point cloud registration. **Required Numpy module**.