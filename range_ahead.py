#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi ,isnan


def scan_callback(msg):
    print("msg.range = %d"%(len(msg.ranges)))
    range_left = msg.ranges[len(msg.ranges) - 1]
    range_ahead = msg.ranges[len(msg.ranges)/2]
    range_right = msg.ranges[0]
    print("range left: %0.1f" % range_left)
    print("range ahead: %0.1f" % range_ahead)
    print("range right: %0.1f" % range_right)


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
            '/scan', LaserScan, self.scan_cb)

        self.scan_data = None
        self.scan_dist_threshold = 1.2    # TODO

        # Scene setting
        self.destination = np.reshape(np.array([10.0, 0.0]), (2, 1))

        self.mline_norm = np.reshape(np.array([0.0,0.0]),(2,1))
        self.mline_norm[1] = - self.destination[0]/np.linalg.norm(self.destination)
        self.mline_norm[0] = - self.destination[1]/np.linalg.norm(self.destination)

        self.mline_threshold = 0.3
        self.hit_point_threshold = 0.4
        self.dest_threshold = 0.4
        # Config
        self.angletolerance = 0.01
        self.angular_speed = 1.0
        self.linear_speed = 0.5

        # State == 0: Walking on the line
        # State == 1: Circling an obstacle
        # State == 2: Reached destination
        # State == 3: Trapped in deadloop
        self.state = 0
        self.hit_point = None
       	self.hit_bef_cnt = 0 # in case hit at the first point

        self.eps_k = 1e-3
        self.eps_l2 = 1e-3

        #self.is_on_m_line = self.initmline()  # TODO: Threshold
        #self.is_at_destination = lambda pos: ((pos - self.destination) ** 2).sum() < self.eps_l2 ** 2

        #rospy.spin()

    """
    Execute this to initialize the m-line
    """
    def scan_cb(self,msg):
    	# print("msg.range = %d"%(len(msg.ranges)))
    	self.scan_data = msg.ranges
        range_left = msg.ranges[len(msg.ranges) - 1]
        range_ahead = msg.ranges[len(msg.ranges)/2]
        range_right = msg.ranges[0]
        # print("range left: %0.1f" % range_left)
        # print("range ahead: %0.1f" % range_ahead)
        # print("ahead isnan:")
        # print(np.isnan(range_ahead))
        # print("range right: %0.1f" % range_right)

    def initmline(self):
        _x1, _y1 = self.get_odom()[0]
        _x2, _y2 = self.destination
        return lambda pos: abs((pos[1] - _y1) * (pos[0] - _x2) - (pos[1] - _y2) * (pos[0] - _x1)) < self.eps_k

    """
    What bug will do in every workloop
    """

    def work(self):
        if self.state == 0:
            self.forward()
        elif self.state == 1:
            self.circle()
        elif self.state == 2:
        	print("Reached Destination!!")
        elif self.state == 3:
        	print("Unable to Reach Destination!!")
        else:
            print("No such state in BUG2 algorithm")

    """
    Main workloop
    """

    def workloop(self):
        while not rospy.is_shutdown() and self.state in (0, 1, 2, 3):
            print("in work loop state = %d" %(self.state))
            self.work()
            rospy.sleep(2)
            print("DEBUG GETHERE")
            self.switch_state()


    def is_on_mline(self):
    	(pos, rot) = self.get_odom()
        nowpos = np.reshape(np.array([pos.x, pos.y]), (1, 2))
        normdis = abs(np.dot(nowpos,self.mline_norm))
        print("now normdis = %f"%(normdis))
        if normdis < self.mline_threshold :
        	print("ON mline")
        	return True
        else:
        	print("OFF mline")
        	return False

    def is_old_hit(self):
    	(pos, rot) = self.get_odom()
        nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
        offset = np.linalg.norm((nowpos - self.hit_point))
        print('offset = %f'%(offset))
        if offset < self.hit_point_threshold:
            print("OLD hit")
            return True
        else:
            print("NOOLD hit")
            return False

    def is_reach(self):
    	(pos, rot) = self.get_odom()
        nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
        offset = np.linalg.norm((nowpos - self.destination))
        print('dest offset = %f'%(offset))
        if offset < self.dest_threshold:
        	return True
        else:
        	return False


    def set_hit_point(self):
    	(pos, rot) = self.get_odom()
        nowpos = np.reshape(np.array([pos.x, pos.y]), (2, 1))
        self.hit_point = nowpos
        self.hit_bef_cnt = 3
        return
    """
    Check state conditions and switch
    If obstacle detected, self.state <- 1, save hit point
    On the line: Call turn_to_dest and set self.state <- 0
    """

    def switch_state(self):
        if self.state == 0:
            if self.is_obstacle():
            	print('switch state')
            	self.set_hit_point()
                self.state = 1
            else:
            	pass

            if self.is_reach():
            	self.state = 2
        elif self.state == 1:
        	if self.is_on_mline():
        		if not self.is_old_hit():
        			self.state = 0
        		else:
        			if self.hit_bef_cnt == 0:
        				self.state = 3
        			else:
        				self.hit_bef_cnt-=1

        	else:
        		self.hit_bef_cnt = 0

        	if self.is_reach():
        		self.state = 2
            # pos = self.get_odom()[0]
            # if ((pos - self.hitpoint) ** 2).sum() < self.eps_l2 ** 2:    # Return to hitpoint
            #     rospy.loginfo("Impossible to reach destination with BUG2 algorithm. Terminating BUG2 turtlebot...")
            #     self.state = 3
            # elif self.is_at_destination(pos):
            #     rospy.loginfo("Arrived at destination!")
            #     self.state = 2
            # elif self.is_on_m_line(pos):
            #     self.turn_to_dest()
            #     self.state = 0
        else:
            print("State %d should not be here"%(self.state))

    def is_obstacle(self):
    	mindis = 100
    	for i in range(len(self.scan_data)):
    		if not np.isnan(self.scan_data[i]):
    			mindis = min(mindis,self.scan_data[i])
    	# print("now the ahead = %f"%(self.scan_data[len(self.scan_data)/2]))
    	print("now min dis = %f" % mindis)
    	if mindis < self.scan_dist_threshold:
    		return True
    	else:
    		return False

    """
    Try to move forward. Only when self.state == 0
    """

    def forward(self):
        # TODO
        try:
            self.turn_to_dest()
        except Exception as e:
        	print("turn_to_dest error: ")
        	print(e.message)
        forward = Twist()
        forward.linear.x = 0.2

        self.cmd_vel_pub.publish(forward)

    def forward_dis(self,dis):
    	print('in the forward_dist')
        rate = 200
        r = rospy.Rate(rate)
        move_cmd = Twist()
        move_cmd.linear.x = self.linear_speed
        ticks = int(dis/self.linear_speed * rate)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)
            r.sleep()
        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)

    """
    Circling the obstacle. Only when self.state == 1
    """

    def circle(self):
        while(self.is_obstacle()):
        	self.rotate(0.5,self.angular_speed)
        self.forward_dis(0.2)
        while(not self.is_obstacle()):
        	self.rotate(-0.3,self.angular_speed)

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
        ticks = int(abs(angle/angspeed) * rate)
        for t in range(ticks):
            self.cmd_vel_pub.publish(move_cmd)  # TODO: null twist?
            r.sleep()
        move_cmd = Twist()
        self.cmd_vel_pub.publish(move_cmd)


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
            
            xaxis = np.reshape(np.array([1, 0]), (1, 2))
            # print("nowangle = ")
            # print(nowangle)
            # print("nowpose =")
            # print(nowpos)
            # print("destpose =")
            # print(destpose)
            # print("xaxis = ")
            # print(xaxis)
            # print(np.dot(xaxis, destpose)/np.linalg.norm(destpose))
            destangle = np.arccos(np.dot(xaxis, destpose)/np.linalg.norm(destpose))
            # print("destangle = %f"% destangle)

            # print(nowpos)
            # print("nowpos[1] = %f"% nowpos[1] )
            
            if(destpose[1] < 0):
                destangle = -destangle

            if abs(nowangle - destangle) < self.angletolerance:
                print('in right direction')
                break

            offset = destangle - nowangle
            try:
                self.rotate(offset, self.angular_speed)
            except Exception as e:
            	print("rotation problem in turn_to_dest")
            	print(e.message)
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
