#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi


def scan_callback(msg):
    range_ahead = msg.ranges[len(msg.ranges)/2]
    print("range ahead: %0.1f" % range_ahead)


class BUG2():
    def __init__(self):
        # ROS init
        rospy.init_node('range_ahead', log_level=rospy.DEBUG)
        self.tf_listener = tf.TransformListener()
        self.world_frame = '/odom'
        self.base_frame = '/base_link'
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback)

        self.scan_data = None
        self.scan_dist_threshold = 1    # TODO

        # Scene setting
        self.destination = np.reshape(np.array([10.0, 0.0]), (2, 1))

        # Config
        self.angletolerance = 0.01
        self.angular_speed = 1.0

        # State == 0: Walking on the line
        # State == 1: Circling an obstacle
        # State == 2: Reached destination
        # State == 3: Trapped in deadloop
        self.state = 0
        self.hitpoint = None

        # scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
        # try:
        # 	self.tf_listener.waitForTransform(self.world_frame,self.base_frame, rospy.Time(), rospy.Duration(1.0))
        # except Exception as e:
        # 	print e.message

        self.eps_k = 1e-3
        self.eps_l2 = 1e-3

        self.is_on_m_line = self.initmline()  # TODO: Threshold
        self.is_at_destination = lambda pos: ((pos - self.destination) ** 2).sum() < self.eps_l2 ** 2

        rospy.spin()

    """
    Execute this to initialize the m-line
    """

    def initmline(self):
        _x1, _y1 = self.get_odom()[0]
        _x2, _y2 = self.destination
        return lambda pos: abs((pos[1] - _y1) * (pos[0] - _x2) - (pos[1] - _y2) * (pos[0] - _x1)) < self.eps_k

    """
    Receive laserscan data
    """

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    """
    What bug will do in every workloop
    """

    def work(self):
        if self.state == 0:
            self.forward()
        elif self.state == 1:
            self.circle()
        else:
            raise ValueError("No such state in BUG2 algorithm")

    """
    Main workloop
    """

    def workloop(self):
        while not rospy.is_shutdown() and self.state in (0, 1):
            self.work()
            rospy.sleep(2)
            self.switch_state()

    """
    Check state conditions and switch
    If obstacle detected, self.state <- 1, save hit point
    On the line: Call turn_to_dest and set self.state <- 0
    """

    def switch_state(self):
        if self.state == 0:
            obstacle = np.array(self.scan_data) < self.scan_dist_threshold
            if obstacle[len(obstacle) // 2]:
                self.state = 1
                self.hitpoint = self.get_odom()[0]
            else:
                pass
        elif self.state == 1:
            pos = self.get_odom()[0]
            if ((pos - self.hitpoint) ** 2).sum() < self.eps_l2 ** 2:    # Return to hitpoint
                rospy.loginfo("Impossible to reach destination with BUG2 algorithm. Terminating BUG2 turtlebot...")
                self.state = 3
            elif self.is_at_destination(pos):
                rospy.loginfo("Arrived at destination!")
                self.state = 2
            elif self.is_on_m_line(pos):
                self.turn_to_dest()
                self.state = 0
        else:
            raise ValueError("State {} should not appear here!".format(self.state))


    """
    Try to move forward. Only when self.state == 0
    """

    def forward(self):
        # TODO
        forward = Twist()
        forward.linear.x = 0.5

        self.cmd_vel_pub.publish(forward)

    """
    Circling the obstacle. Only when self.state == 1
    """

    def circle(self):
        obstacle = np.array(self.scan_data) < self.scan_dist_threshold
        # TODO: Bugs everywhere :p
        if obstacle[-1]:    # Obstacle detected on the right
            # Turn slightly left
            # Move forward
            self.rotate(0.1, self.angular_speed)
        else:
            self.rotate(-0.05, self.angular_speed)
            self.forward()

    """
    Rotate
    """
    def rotate(self, angle, angspeed):
        rate = 200
        r = rospy.Rate(rate)
        move_cmd = Twist()
        if angle < 0:
            move_cmd.angular.z = -angspeed
        else:
            move_cmd.angular.z = angspeed
        ticks = int(abs(angle) * rate)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)  # TODO: null twist?
            r.sleep()
        self.cmd_vel_pub.publish(move_cmd)


    """
    def turn_to_dest(self):
        (pos, rot) = self.get_odom()
        nowangle = rot[2]
        nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
        destpose = self.destination - nowpos
        xaxis = np.reshape(np.array([1, 0]), (2, 1))
        destangle = np.arccos(np.dot(destpose, xaxis)/np.linalg.norm(destpose))

        if(nowpos[1] < 0):
            destangle = -destangle

        print 'nowpos = '
        print nowpos
        print 'nowpos norm '
        print np.linalg.norm(xaxis)
        print 'destangle = %f' % (destangle)
        print 'now angle = %f' % (nowangle)
        if abs(nowangle - destangle) < self.angletolerance:
            print 'in right direction'
            return

        rate = 200
        r = rospy.Rate(rate)
        move_cmd = Twist()
        while abs(nowangle - destangle) > self.angletolerance:
            offset = destangle - nowangle
            print 'nowoffset = %f' % (offset)
            if offset < 0:
                move_cmd.angular.z = -self.angular_speed
            else:
                move_cmd.angular.z = self.angular_speed
            ticks = int(abs(offset)*rate)
            print 'ticks = %d' % (ticks)
            for t in range(ticks):
                self.cmd_vel_pub.publish(move_cmd)
                r.sleep()
            move_cmd = Twist()
            self.cmd_vel_pub.publish(move_cmd)

            (pos, rot) = self.get_odom()
            nowangle = rot[2]
            nowpos = np.array([pos.x, pos.y])
            nowpos.reshape(2, 1)
            destpose = self.destination - nowpos
            xaxis = np.array([1, 0])
            xaxis.reshape(2, 1)
            destangle = np.arccos(
                np.dot(destpose, xaxis)/np.linalg.norm(destpose))

            if(nowpos[1] < 0):
                destangle = -destangle
            rospy.sleep(abs(offset))
    """

    """
    Turn to destination
    """

    def turn_to_dest(self):
        maxiter = 100
        while maxiter > 0:
            (pos, rot) = self.get_odom()
            nowangle = rot[2]
            nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
            destpose = self.destination - nowpos
            xaxis = np.reshape(np.array([1, 0]), (2, 1))
            destangle = np.arccos(
                np.dot(destpose, xaxis)/np.linalg.norm(destpose))

            if(nowpos[1] < 0):
                destangle = -destangle

            if abs(nowangle - destangle) < self.angletolerance:
                print('in right direction')
                break

            offset = destangle - nowangle

            self.rotate(offset, self.angular_speed)
            maxiter -= 1

    """
    Get current position (w/ odom) in world frame
    """

    def get_odom(self):
        try:
            self.tf_listener.waitForTransform(
                self.world_frame, self.base_frame, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, self.base_frame, rospy.Time(0))
            eulr = tf.transformations.euler_from_quaternion(rot)
        except Exception as e:
            rospy.loginfo(e.message)
            rospy.loginfo("TF Exception")
        return (Point(*trans), eulr)


if __name__ == '__main__':
    try:
        bug = BUG2()
        bug.workloop()
    except Exception as e:
        print(e.message)
        rospy.loginfo("BUG2 node terminated")
