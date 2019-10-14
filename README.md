# WHAC

## Components
1. LIDAR System (rplidar + our lidar code)
2. ZED Mini + ORB-SLAM2 (orb-slam2 + our slam code)
3. Motion Planning
4. Robot

## Code
In total there will be 6 ROS packages. Two for LIDAR, two for SLAM, one for motion planning and one for robot control.

## Run
Run ZED: roslaunch zed_wrapper zed.launch

Run ORB-SLAM2: rosrun ORB_SLAM2 RGBD src/slam/ORB_SLAM2/Vocabulary/ORBvoc.txt src/slam/ORB_SLAM2/Examples/ROS/ORB_SLAM2/zedm.yaml

## Important Links
ORB-SLAM2 Paper: https://arxiv.org/pdf/1610.06475.pdf

ZED SLAM implementation: https://github.com/yifenghuang/ZSLAM_TX2

ORB-SLAM2 code: https://github.com/raulmur/ORB_SLAM2
