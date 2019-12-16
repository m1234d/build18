#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, MultiArrayLayout, UInt8MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import cv2

#TODO: Subscribe to orb slam topics and generate wall grid here, then add to whac_slam.launch

pose = None
points = None
pub = None
def pointcloud2_to_arr(msg):
    type_list = [(f.name, np.float32) for f in msg.fields]
    arr = np.fromstring(msg.data, type_list)
    return np.reshape(arr, (msg.width))

def callback_points(data):
    global points
    arr = pointcloud2_to_arr(data)
    points = arr
    publish()

def callback_pose(data):
    global pose
    pose = data.pose
    print(pose)

def publish():
    global pub
    global pose
    size = 5.0
    step_size = .01
    grid = np.zeros((int(size/step_size), int(size/step_size))).astype(np.uint8)
    # small_x = min([p[0] for p in points])
    # large_x = max([p[0] for p in points])

    # small_y = min([p[1] for p in points])
    # large_y = max([p[1] for p in points])

    # new_points = [()]
    # for p in points:
    for p in points:
        x = int(p[0] / step_size) + int(size/(2*step_size))
        y = int(p[1] / step_size) + int(size/(2*step_size))
        if x >= grid.shape[0] or y >= grid.shape[1] or x < 0 or y < 0:
            continue
        grid[x, y] = 255
    grid[int(pose.position.x/step_size) + int(size/(2*step_size)), int(pose.position.y/step_size) + int(size/(2*step_size))] = 127
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 600, 600)
    cv2.imshow('image', grid)
    cv2.waitKey(1)
    # grid = np.reshape(grid, (grid.shape[0]*grid.shape[1])).tolist()
    # msg = UInt8MultiArray()
    # msg.layout = MultiArrayLayout()
    # msg.layout.data_offset = step_size
    # msg.data = grid
    # pub.publish(msg)


    
def listener():
    global pose
    global points
    global pub
    pose = Pose()
    points = np.zeros((1))
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('slam_listen', anonymous=True)

    rospy.Subscriber("/orb_slam2_rgbd/map_points", PointCloud2, callback_points)
    rospy.Subscriber("/orb_slam2_rgbd/pose", PoseStamped, callback_pose)

    # spin() simply keeps python from exiting until this node is stopped
    pub = rospy.Publisher('/whac_slam/grid', UInt8MultiArray, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()