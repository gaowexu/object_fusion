# Object Fusion C++ SDK For Autonomous Driving

*Author: Gaowei Xu (gaowexu1991@gmail.com)*


### 1. Introduction

In this repository, a typical object fusion SDK based on C++ is implemented and it uses the Bazel toolchain for compilation and unit testing. The core of this repository exploits the linear Kalman model to predict and fuse the motion state of the detected objects (from LiDAR 3D detection module[1-3] or BEV visual 3D detection module[4-6], etc.). For association part, both Euclidean distance and intersection-over-union (IoU) metrics are applied to measure the distance between objects. This SDK can be used or generalized for multi-sensor object fusion in highway scenarios (such as NOP, LCC, LKA, ACC). In this repository, the following features are supported:

- Kalman process C++ implementation.

- Objects association with Munkres algorithm[7].

- Bazel build and unit test for each module of object fusion SDK.
- User guide for how to setup bazel toolchain from scratch.


### 2. Kalman & Object Fusion

![Architecture_Of_Object_Fusion](./assets/architect.png)

In this repository, we formulate the Kalman process using LiDAR 3D object detection (single sensor input) as an example. First, the 3D objects detected by the lidar are transformed from the ego-vehicle-rear-axis coordinate system to the odometry coordinate system. Then Kalman prediction, agent association, state fusion (including creation of new tracks) are cascaded. Finally, these fused objects (i.e., optimal estimations) are transformed back to the ego-vehicle-rear-axis coordinate system and published via ROS messages.

### 3. Bazel Build & Unit Test

#### 3.1 Install Bazel
Firstly launch an ECS instance `ecs.s6-c1m4.xlarge` in [aliyun](https://ecs-buy.aliyun.com/) and choose `Ubuntu 20.04 64-bit` as operating system image. Wait minutes and login the ECS server via ssh.

After successfully login to the ECS server, please run the following commands:
```
sudo apt-get update
sudo apt-get upgrade
```

Following the user guide to install [bazel](https://bazel.build/install/ubuntu?hl=zh-cn) in Ubuntu 20.04:
```
sudo apt install g++ unzip zip
```
Download [bazel installion script](https://github.com/bazelbuild/bazel/releases):
```
wget -c https://github.com/bazelbuild/bazel/releases/download/6.2.0/bazel-6.2.0-installer-linux-x86_64.sh
chmod a+x bazel-6.2.0-installer-linux-x86_64.sh
./bazel-6.2.0-installer-linux-x86_64.sh --user
export PATH="$PATH:$HOME/bin"
```

Then run command ```bazel --version```, it will display the below message in terminal:
```
root@iZbp19iv7rc5oi5ssvezt5Z:~# bazel --version
bazel 6.2.0
```

#### 3.2 Virtual Third-Party Packages Server Setup
In order to simulate the third-party packages server, we use the local ECS machine as server, we firstly install apache to play as the role of this-party dependencies server.
```
sudo apt-get install apache2
```

Upload this repo to the ECS and copy third-party dependencies to `/var/www/html/` directory:
```
cd assets
sudo cp * /var/www/html/.
```

#### 3.3 Build & Test of Third-Party Packages
> bazel build //third_party/google_test:gtest

The running result is shown below:
```
root@iZbp19iv7rc5oi5ssvezt5Z:~/object_fusion# bazel build //third_party/google_test:gtest
INFO: Analyzed target //third_party/google_test:gtest (2 packages loaded, 55 targets configured).
INFO: Found 1 target...
Target @google_test//:gtest up-to-date:
  bazel-bin/external/google_test/libgtest.a
  bazel-bin/external/google_test/libgtest.so
INFO: Elapsed time: 7.949s, Critical Path: 5.47s
INFO: 18 processes: 3 internal, 15 linux-sandbox.
INFO: Build completed successfully, 18 total actions
```

> bazel build //third_party/google_test:gtest_main

The running result is shown below:
```
root@iZbp19iv7rc5oi5ssvezt5Z:~/object_fusion# bazel build //third_party/google_test:gtest_main
INFO: Analyzed target //third_party/google_test:gtest_main (0 packages loaded, 3 targets configured).
INFO: Found 1 target...
Target @google_test//:gtest_main up-to-date:
  bazel-bin/external/google_test/libgtest_main.a
  bazel-bin/external/google_test/libgtest_main.so
INFO: Elapsed time: 1.342s, Critical Path: 1.14s
INFO: 6 processes: 3 internal, 3 linux-sandbox.
INFO: Build completed successfully, 6 total actions
```

> bazel test //third_party/google_test/test:gtest_unit_test --test_output=all

The running result is shown below:
```
root@iZbp19iv7rc5oi5ssvezt5Z:~/object_fusion# bazel test //third_party/google_test/test:gtest_unit_test --test_output=all
INFO: Analyzed target //third_party/google_test/test:gtest_unit_test (0 packages loaded, 0 targets configured).
INFO: Found 1 test target...
INFO: From Testing //third_party/google_test/test:gtest_unit_test:
==================== Test output for //third_party/google_test/test:gtest_unit_test:
Running main() from gmock_main.cc
[==========] Running 1 test from 1 test suite.
[----------] Global test environment set-up.
[----------] 1 test from HelloTest
[ RUN      ] HelloTest.BasicAssertions
[       OK ] HelloTest.BasicAssertions (0 ms)
[----------] 1 test from HelloTest (0 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
================================================================================
Target //third_party/google_test/test:gtest_unit_test up-to-date:
  bazel-bin/third_party/google_test/test/gtest_unit_test
INFO: Elapsed time: 1.181s, Critical Path: 1.02s
INFO: 4 processes: 1 internal, 3 linux-sandbox.
INFO: Build completed successfully, 4 total actions
//third_party/google_test/test:gtest_unit_test                           PASSED in 0.1s
```



Eigen build:
> bazel build //third_party/eigen:eigen

The running result is shown below:
```
root@iZbp19iv7rc5oi5ssvezt5Z:~/object_fusion# bazel build //third_party/eigen:eigen
INFO: Analyzed target //third_party/eigen:eigen (6 packages loaded, 788 targets configured).
INFO: Found 1 target...
Target @eigen//:eigen up-to-date (nothing to build)
INFO: Elapsed time: 2.353s, Critical Path: 0.05s
INFO: 1 process: 1 internal.
INFO: Build completed successfully, 1 total action
```


#### 3.4 Build & Unit Test of Object Fusion SDK

> bazel test --test_output=all //modules/test:object_fusion_unit_test

```
INFO: Analyzed target //modules/test:object_fusion_unit_test (0 packages loaded, 0 targets configured).
INFO: Found 1 test target...
Target //modules/test:object_fusion_unit_test up-to-date:
  bazel-bin/modules/test/object_fusion_unit_test
INFO: Elapsed time: 4.626s, Critical Path: 4.47s
INFO: 3 processes: 1 internal, 2 linux-sandbox.
INFO: Build completed successfully, 3 total actions
PASSED: //modules/test:object_fusion_unit_test (see /home/xuzhu/.cache/bazel/_bazel_xuzhu/2ed4ae248c27c3f1258ed98a2867df24/execroot/object_fusion/bazel-out/k8-fastbuild/testlogs/modules/test/object_fusion_unit_test/test.log)
INFO: From Testing //modules/test:object_fusion_unit_test
==================== Test output for //modules/test:object_fusion_unit_test:
Running main() from gmock_main.cc
[==========] Running 1 test from 1 test suite.
[----------] Global test environment set-up.
[----------] 1 test from perception
[ RUN      ] perception.BEVObjectFusionUnitTest
[       OK ] perception.BEVObjectFusionUnitTest (334 ms)
[----------] 1 test from perception (334 ms total)

[----------] Global test environment tear-down
[==========] 1 test from 1 test suite ran. (334 ms total)
[  PASSED  ] 1 test.
================================================================================
//modules/test:object_fusion_unit_test                          (cached) PASSED in 0.4s

INFO: Build completed successfully, 3 total actions
```

### 4. Liscene




### 5. Reference

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krahenbuhl. "Center-based 3d object detection and tracking." Proceedings of the IEEE/CVF conference on computer vision and pattern recognition. 2021.

[2] Lang, Alex H., et al. "Pointpillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF conference on computer vision and pattern recognition. 2019.

[3] Liu, Zhijian, et al. "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation." arXiv preprint arXiv:2205.13542 (2022).

[4] Huang J, Huang G, Zhu Z, et al. BEVDet: High-performance Multi-camera 3D Object Detection in Bird-Eye-View[J]. arXiv e-prints, 2021: arXiv: 2112.11790.

[5] Li Y, Ge Z, Yu G, et al. BEVDepth: Acquisition of Reliable Depth for Multi-view 3D Object Detection[J]. arXiv e-prints, 2022: arXiv: 2206.10092.

[6] Li, Yangguang, et al. "Fast-BEV: A Fast and Strong Bird's-Eye View Perception Baseline." arXiv preprint arXiv:2301.12511 (2023).

[7] http://www.columbia.edu/~cs2035/courses/ieor6614.S16/GolinAssignmentNotes.pdf

[8] Bazel user guide: https://bazel.build/?hl=zh-cn