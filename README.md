# build18
WHAC!

source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0 (and /dev/ttyACM0)
urdf:
hector_mapping/mapping_launch.default
for laser and base link and odom to mapping

mapping:

roslaunch rplidar_ros rplidar.launch
roslaunch roboclaw_node roboclaw.launch
roslaunch hector_slam_launch tutorial.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

saving the map:
rosrun  map_server map_saver -f [map name here]

you can close everything now


localization:

launch lidar
launch roboclaw
launch hector_slam
rosrun map_server map_server [name of map.yaml] /map:=[any name here except for 'map']

in rviz: 
untick map (on the left panel)
click add, by topic, and click the new map (whatever you had named it)
