#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
import cv2
import numpy as np
import cv_bridge
import yaml


global shutdown_requested


class FindCodes(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done', 'move_to_target'])
        self.callbacks = callbacks
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:
            time.sleep(1)
            # TODO: rotate 360 degrees
            # Temporary ------------------------------
            if self.callbacks.target_position is not None:
                return 'move_to_target'
        return 'done'


class MoveToTarget(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done', 'stop'])
        self.callbacks = callbacks
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()

    def execute(self, userdata):
        global shutdown_requested

        print("Turning")
        turning = True
        previous_difference = None
        while turning:
            desired_heading = get_bearing_between_points(self.callbacks.bot_position, self.callbacks.target_position)
            difference = minimum_angle_between_headings(desired_heading, self.callbacks.bot_heading)

            if previous_difference is None:
                self.twist.angular.z = (0.5-(difference <= 0))*2*0.4
            else:
                if abs(difference) < 1:
                    turning = False
                    self.twist.angular.z = 0
                else:
                    self.twist.angular.z = (0.5-(difference <= 0))*2*0.4

            self.cmd_vel_pub.publish(self.twist)

            if previous_difference != difference:
                previous_difference = difference

            if shutdown_requested:
                return 'done'

        time.sleep(2)
        self.callbacks.stop_updating_tags = True

        print("Moving forward")
        moving_forward = True
        while True:

            desired_heading = get_bearing_between_points(self.callbacks.bot_position, self.callbacks.target_position)
            difference = minimum_angle_between_headings(desired_heading, self.callbacks.bot_heading)

            if previous_difference is None:
                self.twist.angular.z = (0.5 - (difference <= 0)) * 2 * 0.4
            else:
                if abs(difference) < 1:
                    turning = False
                    self.twist.angular.z = 0
                else:
                    self.twist.angular.z = (0.5 - (difference <= 0)) * 2 * 0.4

            bp = self.callbacks.bot_position
            tp = self.callbacks.real_target_position

            if previous_difference != difference:
                t = (bp.x - tp.x) ** 2 + (bp.y - tp.y) ** 2
                print(str(self.callbacks.target_heading) + " " + str(self.callbacks.bot_heading))
                print("distance:"+str(t)+" angleD:"+str(difference))
                previous_difference = difference

            if math.sqrt((bp.x - tp.x) ** 2 + (bp.y - tp.y) ** 2) < 0.3:
                moving_forward = False
                return 'stop'

            if shutdown_requested:
                return 'done'

            self.twist.linear.x = 0.25
            self.cmd_vel_pub.publish(self.twist)
            self.callbacks.stop_updating_tags = False


class Stop(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done', 'move_to_target', 'find_codes'])
        self.callbacks = callbacks
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        print("Robot has stopped")
        while not shutdown_requested:

            start = time.time()
            if time.time() - start > 10:
                return 'move_to_target'  # -----------------------
        return 'done'


class Callbacks:
    def __init__(self):
        self.target_position = None
        self.target_heading = None
        self.bot_heading = None
        self.bot_position = None
        self.real_target_position = None

        self.ids = [Tag(1), Tag(2), Tag(3), Tag(4), Tag(5), Tag(6), Tag(7), Tag(8), Tag(9), Tag(10)]
        self.found_ids = []

        self.stop_updating_tags = False
        self.i = 0

    def qr_callback(self, msg):
        if len(msg.markers) > 0:
            for marker in msg.markers:
                if marker.id == 5 and self.stop_updating_tags is False:
                    yaw = euler_from_quaternion([
                        marker.pose.pose.orientation.x,
                        marker.pose.pose.orientation.y,
                        marker.pose.pose.orientation.z,
                        marker.pose.pose.orientation.w
                    ])[2]
                    # TODO: add distance in front of qr code
                    self.ids[5-1].add_to_headings(math.degrees(yaw)+180 - 90)
                    self.target_heading = self.ids[5-1].get_heading()
                    #self.target_position = marker.pose.pose.position
                    p = marker.pose.pose.position
                    p.x += 0.4
                    self.target_position = marker.pose.pose.position
                    self.real_target_position = p
                   #self.target_position = get_point(marker.pose.pose.position, 0.4, self.target_heading)

                    #self.found = True

    def odom_callback(self, msg):
        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        self.bot_heading = (yaw + math.pi) * (180 / math.pi)
        self.bot_position = msg.pose.pose.position


class Tag:
    def __init__(self, id):
        self.id = id
        self.checked = False
        self.positions = []
        self.headings = []

    def add_to_positions(self, position):
        if len(self.positions) > 10:
            del self.positions[0]
        self.positions.append(position)

    def add_to_headings(self, heading):
        if len(self.headings) > 10:
            del self.headings[0]
        self.headings.append(heading)

    def get_position(self):
        assert len(self.positions) > 0
        for axis in [1, 2]:
            print("hello")
        return self.average_position

    def get_heading(self):
        assert len(self.headings) > 0
        return sum(self.headings)/len(self.headings)


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
        heading_difference = -heading_difference
    return heading_difference


def get_bearing_between_points(a, b):
    dx = b.x - a.x
    dy = b.y - a.y
    bearing = math.degrees(math.atan2(dy, dx))
    return bearing+180


def get_point(point, distance, heading):
    angle = math.radians(heading-180)
    point.x = point.x + distance * math.cos(angle)
    point.y = point.y + distance * math.sin(angle)
    return point


def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True


def main():
    global shutdown_requested
    shutdown_requested = False

    callbacks = Callbacks()

    image_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callbacks.qr_callback, queue_size=1)
    odom_sub = rospy.Subscriber("odom", Odometry, callbacks.odom_callback)

    rospy.init_node('box_map')
    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('FIND_CODES', FindCodes(callbacks),
                               transitions={'done': 'DONE', 'move_to_target': 'MOVE_TO_TARGET'})
        smach.StateMachine.add('MOVE_TO_TARGET', MoveToTarget(callbacks),
                               transitions={'done': 'DONE', 'stop': 'STOP'})
        smach.StateMachine.add('STOP', Stop(callbacks),
                               transitions={'done': 'DONE',
                                            'move_to_target': 'MOVE_TO_TARGET',
                                            'find_codes': 'FIND_CODES'})

    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()