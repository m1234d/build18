import rospy
from std_msgs.msg import String, Bool, Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from collections import namedtuple
import numpy as np
from math import sin, cos
import tf


class SamplingPlanner:
    def __init__(self):
        Params = namedtuple('Params',
                            " ".join([
                                'bubbleRadius',  # m
                                'wheelbaseWidth',  # m
                                'maxSpeed',  # m/s
                                'maxAngle',  # rad
                                'horizonTime',  # s
                                'numTrajectories',
                                'numPts',
                                'frequency']))
        self.params = Params(
            bubbleRadius=rospy.get_param("~bubbleRadius"),
            wheelbaseWidth=rospy.get_param("~wheelbaseWidth"),
            maxSpeed=rospy.get_param('~maxSpeed'),
            maxAngle=rospy.get_param("~maxAngle"),
            horizonTime=rospy.get_param("~horizonTime"),
            numTrajectories=rospy.get_param("~numTrajectories"),
            numPts=rospy.get_param("~numPts"),
            frequency=rospy.get_param("~frequency")
        )

        self.x = [0, 0, 0]
        self.obstacles = [[1, 0]]

        if self.params.numTrajectories % 2 == 0:
            rospy.logerr("Error! Number of trajectories must be odd!")

        self.trajPubs = []
        for i in range(self.params.numTrajectories):
            self.trajPubs.append(rospy.Publisher("traj_candidate{}".format(i), Path, queue_size=10))

    def computeTrajectories(self):
        trajectories = []
        inputs = []

        for i in range(-self.params.numTrajectories / 2 + 1, self.params.numTrajectories / 2 + 1):
            angVel = float(i) * self.params.maxAngle
            xVel = self.calcXVal(angVel)
            dt = 1.0 / self.params.numPts * self.params.horizonTime

            x = [self.x]  # x,y,th
            u = [xVel, angVel]

            for n in range(self.params.numPts):
                x.append(self.motion(x[-1], u, dt))

            trajectories.append(x)
            inputs.append(u)

        return trajectories, inputs

    def removeCollidingPaths(self, trajectories, inputs):
        newTrajectories = []
        newInputs = []

        obstacles = self.obstacles
        threshold = self.params.bubbleRadius ** 2

        for i in range(len(inputs)):
            traj = trajectories[i]
            input = inputs[i]
            cull = False
            for pt in traj:
                dists = [(o[0] - pt[0]) ** 2 + (o[1] - pt[1]) ** 2 for o in obstacles]
                if min(dists) < threshold:
                    cull = True
                    break
            if not cull:
                newTrajectories.append(traj)
                newInputs.append(input)

        return newTrajectories, newInputs

    def calcXVal(self, angVel, forward=True):
        wheelSpeed = float(self.params.wheelbaseWidth) / 2 * angVel
        xSpeed = max(self.params.maxSpeed - abs(wheelSpeed), 0)
        return xSpeed if forward else -xSpeed

    def motion(self, x, u, dt):
        res = [0, 0, 0]
        res[0] = x[0] + u[0] * cos(x[2]) * dt
        res[1] = x[1] + u[0] * sin(x[2]) * dt
        res[2] = x[2] + u[1] * dt
        return res

    def update(self, timerEvent):
        (trajectories, inputs) = self.computeTrajectories()
        (trajectories, inputs) = self.removeCollidingPaths(trajectories, inputs)
        self.publishTrajectories(trajectories)

    def publishTrajectories(self, trajectories):
        for i in range(len(trajectories)):
            path = Path()
            h = Header()
            path.header = h
            h.frame_id = 'odom'
            h.stamp = rospy.Time.now()
            for pt in trajectories[i]:
                p = PoseStamped()
                p.header = h
                p.pose = Pose()
                p.pose.position = Point(pt[0], pt[1], 0)
                q = tf.transformations.quaternion_from_euler(0, 0, pt[2])
                p.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                path.poses.append(p)
            self.trajPubs[i].publish(path)

    def laserCallback(self):
        pass

    def start(self):
        rospy.Timer(rospy.Duration(1.0 / self.params.frequency), self.update)
        rospy.spin()


def main():
    rospy.init_node('sampling_planner')
    p = SamplingPlanner()
    p.start()
