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


class bug2():
    def __init__(self):
        # ROS init
        rospy.init_node('range_ahead', log_level=rospy.DEBUG)
        self.tf_listener = tf.TransformListener()
        self.world_frame = '/odom'
        self.base_frame = '/base_link'
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        # Scene setting
        self.destination = np.reshape(np.array([10.0, 0.0]), (2, 1))

        # Config
        self.angletolerance = 0.01
        self.angular_speed = 1.0
        
        # State == 0: Walking on the line
        # State == 1: Circling an obstacle
        self.state = 0

        # scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
        # try:
        # 	self.tf_listener.waitForTransform(self.world_frame,self.base_frame, rospy.Time(), rospy.Duration(1.0))
        # except Exception as e:
        # 	print e.message

        self.is_on_the_line = self.initline(1e-3)  # TODO: Threshold

        self.workloop()

        rospy.spin()

    """
    Execute this to initialize the line
    """
    def initline(self, eps):
        _x1, _y1 = self.get_odom()[0]
        _x2, _y2 = self.destination
        return lambda x, y: abs((y - _y1) * (x - _x2) - (y - _y2) * (x - _x1)) < eps

    """
    What bug will do in every workloop
    """
    def work(self):
        if self.state == 0
            self.forward()
        elif self.state == 1:
            self.circle()
        else:
            raise ValueError("No such state in BUG2 algorithm")

    """
    Main workloop
    """
    def workloop(self):
        while not rospy.is_shutdown():
            self.work()
            rospy.sleep(2)
    
    """
    Try to move forward. Only when self.state == 0
    If obstacle detected, self.state <- 1
    Otherwise just move forward
    """
    def forward(self):
        # TODO
        forward = Twist()
        forward.linear.x = 0.5


    """
    Circling the obstacle till self.is_on_the_line(pos) is True. Only when self.state == 1
    Circling: Spin once
    On the line: Call turn_to_dest and set self.state <- 0
    """
    def circle(self):
        # TODO
        pass

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
        rate = 200
        r = rospy.Rate(rate)
        move_cmd = Twist()
        maxiter = 100
        while maxiter > 0:
            (pos, rot) = self.get_odom()
            nowangle = rot[2]
            nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
            destpose = self.destination - nowpos
            xaxis = np.reshape(np.array([1, 0]), (2, 1))
            destangle = np.arccos(np.dot(destpose, xaxis)/np.linalg.norm(destpose))

            if(nowpos[1] < 0):
                destangle = -destangle	

            if abs(nowangle - destangle) < self.angletolerance:
                print('in right direction')
                break

            offset = destangle - nowangle
            print('nowoffset = %f' % (offset))
            if offset < 0:
                move_cmd.angular.z = -self.angular_speed
            else:
                move_cmd.angular.z = self.angular_speed
            ticks = int(abs(offset)*rate)
            print('ticks = %d' % (ticks))
            for t in range(ticks):
                self.cmd_vel_pub.publish(move_cmd)	# TODO: null twist?
                r.sleep()
            self.cmd_vel_pub.publish(move_cmd)

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
        bug2()
    except Exception as e:
        print(e.message)
        rospy.loginfo("BUG2 node terminated")
