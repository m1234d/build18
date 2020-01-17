import rospy

from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection
from tf import TransformerROS


class Laser2Cloud:
    def __init__(self):
        self.tfr = TransformerROS()

        self.projector = LaserProjection()

        self.outputTopic = rospy.get_param('output_topic', 'cloud')
        self.inputTopic = rospy.get_param('input_topic', 'scan')
        self.outputFrame = rospy.get_param('output_frame', 'base_link')

        self.publisher = rospy.Publisher(self.outputTopic, PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber(self.inputTopic, LaserScan, self.laser_callback)

    def laser_callback(self, data):
        cloud = self.projector.projectLaser(data)
        self.publisher.publish(cloud)


def main():
    rospy.init_node('laser_to_cloud')
    Laser2Cloud()
    rospy.spin()
