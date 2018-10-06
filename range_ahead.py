#!/usr/bin/env python
import rospy
import numpy as np
import scipy.linalg as linalg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan

class BUG2():
    def __init__(self):
        # ROS init
        rospy.init_node('range_ahead') #, log_level=rospy.DEBUG)
        self.tf_listener = tf.TransformListener()
        self.world_frame = '/odom'
        self.base_frame = '/base_link'
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel_mux/input/teleop', Twist, queue_size=10000)
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_cb)

        self.scan_data = None

        # Scene setting
        self.destination = np.array([10.0, 0.0])
        self.destnorm = linalg.norm(self.destination)

        self.mline_vec = self.destination / self.destnorm                           # MLine vector
        self.mline_norm = np.matmul(np.array([[0, -1], [1, 0]]), self.mline_vec)    # MLine norm vec

        # Config
        self.angletolerance = 0.08
        self.angular_speed = 0.5
        self.linear_speed = 1.0

        self.linear_step = 0.5

        # State == 0: Walking on the line
        # State == 1: Circling an obstacle
        # State == 2: Reached destination
        # State == 3: Trapped in deadloop
        self.state = 0
        self.hit_point = np.array([0, 0])
        self.hit_bef_cnt = 0 

        self.eps_l2 = 0.4
        self.mline_threshold = 0.3                                                  # Dist to MLine
        self.scan_dist_threshold = 1.5

        self.is_on_mline = lambda pos: abs(pos.dot(self.mline_norm)) < self.mline_threshold and (self.hit_point.dot(self.mline_vec) <= pos.dot(self.mline_vec) <= self.destnorm)
        self.is_at_hitpoint = lambda pos: ((pos - self.hit_point) ** 2).sum() < self.eps_l2 ** 2
        self.is_at_destination = lambda pos: ((pos - self.destination) ** 2).sum() < self.eps_l2 ** 2
        self.is_obstacle = lambda: np.nanmin(self.scan_data) < self.scan_dist_threshold if not all(np.isnan(self.scan_data)) else False

        # rospy.spin()

    def scan_cb(self, msg):
        # if not all(np.isnan(msg.ranges)):
        self.scan_data = np.array(msg.ranges)
        """
        range_left = msg.ranges[-1]
        range_ahead = msg.ranges[len(msg.ranges)/2]
        range_right = msg.ranges[0]
        print("{:.2f}, {:.2f}, {:.2f}".format(range_left, range_ahead, range_right))
        """

    """
    What bug will do in every workloop
    """
    def work(self):
        if self.state == 0:
            self.forward()
        elif self.state == 1:
            self.circle()
        else:
            raise ValueError("No such state in BUG2 algorithm, or state {} should not reach here".format(self.state))

    """
    Main workloop
    """
    def workloop(self):
        while not rospy.is_shutdown() and self.state in (0, 1):
            print("State = %d" % (self.state))
            self.work()
            self.switch_state()
        if self.state == 2:
            print("State 2: Turtlebot is at its destination. Congratulations!")
        elif self.state == 3:
            print("State 3: Turtlebot cannot find a way to its destination QAQ")
        else:
            print("rospy shutdown")

    """
    Check state conditions and switch
    If obstacle detected, self.state <- 1, save hit point
    On the line: self.state <- 0
    """
    def switch_state(self):
        pos = self.get_odom()[0]
        if self.is_at_destination(pos):
            self.state = 2
            return

        if self.state == 0:
            if self.is_obstacle():
                print('Obstacle detected, switch to Circle Mode')
                self.set_hit_point()
                self.state = 1
            else:
                pass
        elif self.state == 1:
            msg = self.scan_data
            range_left = msg[-1]
            range_ahead = msg[len(msg)/2]
            range_right = msg[0]
            print("{:.2f}, {:.2f}, {:.2f}".format(range_left, range_ahead, range_right))
            print("Current POS = {}".format(pos))
            print("Hitpoint = {}".format(self.hit_point))
            print("MLINE = {}, n = {}".format(self.mline_vec, self.mline_norm))
            print("self.is_on_mline(pos) = {}".format(self.is_on_mline(pos)))
            print("self.is_at_hitpoint(pos) = {}".format(self.is_at_hitpoint(pos)))
            print("self.hit_bef_cnt = {}".format(self.hit_bef_cnt))
            if self.is_on_mline(pos):
                if not self.is_at_hitpoint(pos):
                    print("Switch to Forward Mode")
                    self.state = 0
                else:
                    if self.hit_bef_cnt == 0:
                        self.state = 3
                    else:
                        self.hit_bef_cnt -= 1
            else:
                self.hit_bef_cnt = 0
        elif self.state in (2, 3):
            raise ValueError("State {} should not reach here!".format(self.state))
        else:
            raise ValueError("No such state {}".format(self.state))

    """
    Try to move forward. Only when self.state == 0
    """
    def forward(self):
        try:
            dist = self.turn_to_dest()
        except Exception as e:
            print("turn_to_dest error: ")
            print(e.message)
            return
        
        if dist > self.linear_step * 1.5:
            self.cmd_forward(self.linear_step * 1.5)
            rospy.sleep(0.4)
        else:
            self.cmd_forward(self.linear_step)
            rospy.sleep(0.3)

    """
    Circling the obstacle. Only when self.state == 1
    """
    def circle(self):
        cnt = 2
        while(self.is_obstacle()):
            self.cmd_rotate(0.1)
            cnt += 1
        self.cmd_forward(self.linear_step)
        while(not self.is_obstacle()):
            self.cmd_rotate(-0.05)
            cnt += 1
        self.cmd_rotate(0.05)
        rospy.sleep(min(cnt * 0.05, 0.5))

    """
    Record hit point
    """
    def set_hit_point(self):
        self.hit_point = self.get_odom()[0]
        self.hit_bef_cnt = 3

    """
    Turn to destination
    """
    def turn_to_dest(self):
        maxiter = 100
        while maxiter > 0:
            (pos, rot) = self.get_odom()
            if pos is None:
                continue
            nowangle = rot[2]
            nowpos = pos
            destpose = self.destination - nowpos
            destangle = np.arctan2(*(destpose[::-1]))
            if abs(nowangle - destangle) < self.angletolerance:
                print('in right direction')
                break
            offset = destangle - nowangle
            try:
                print("Adjusting moving direction")
                self.cmd_rotate(offset)
            except Exception as e:
                print("rotation problem in turn_to_dest")
                print(e.message)
            maxiter -= 1
        return linalg.norm(destpose)

    """
    Basic commands
    """

    def cmd_rotate(self, angle):
        angspeed = self.angular_speed
        rate = 20
        r = rospy.Rate(rate)
        move_cmd = Twist()
        if angle < 0:
            move_cmd.angular.z = -angspeed
        else:
            move_cmd.angular.z = angspeed
        ticks = int(abs(angle/angspeed) * rate)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)  # TODO: null twist?
            r.sleep()
        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)

    def cmd_forward(self, dis):
        # print('in the forward_dist')
        rate = 20
        r = rospy.Rate(rate)
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        ticks = int(dis / self.linear_speed * rate)
        for _ in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)
            r.sleep()
        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)

    """
    Get current position (w/ odom) in world frame
    """

    def get_odom(self):
        try:
            trans, rot = None, None
            self.tf_listener.waitForTransform(
                self.world_frame, self.base_frame, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame, self.base_frame, rospy.Time(0))
            eulr = tf.transformations.euler_from_quaternion(rot)
        except Exception as e:
            rospy.loginfo(e.message)
            rospy.loginfo("TF Exception")
        # return (Point(*trans), eulr)
        if trans is None:
            return None, None
        p = Point(*trans)
        return (np.array([p.x, p.y]), eulr)


if __name__ == '__main__':
    print("Successfuly imported all packages")
    try:
        bug = BUG2()
        bug.workloop()
    except Exception as e:
        print(e.message)
        rospy.loginfo("BUG2 node terminated")
