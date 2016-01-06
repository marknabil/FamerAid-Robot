#!/usr/bin/env python

""" farm_aid.py - Version 1.5.7 2015-12-20


    Created for the Robotics Project module in the master of computer vision
    in Universite De Bourgogne.

      
"""

import rospy
import actionlib
import smach
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String

import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

from rbx2_tasks.task_setup_Grape import *

from ar_track_alvar.msg import AlvarMarkers

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import shlex
import psutil

import subprocess
import os
import signal
import random
import numpy as np


from vision import DetectTag
        
from nav import Rotate


class top_state(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])
    def execute(self):
        rospy.loginfo('Executing state Start Robot')
        rospy.sleep(1)
        return 'start'

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach', anonymous=False)
                
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        self.bridge = CvBridge()

        # A list to hold then navigation waypoints
        nav_states = list()
        
        # Turn the waypoints into SMACH states
        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(90.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)


        home_goal=MoveBaseGoal()
        home_goal.target_pose.header.frame_id='map'
        home2=Pose(Point(0.0,0.0,0.0),Quaternion(0,0,0,1))
        home_goal.target_pose.pose=home2
        move_base_state_home=SimpleActionState('move_base', MoveBaseAction, goal=home_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(120.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
        home_state=move_base_state_home

        # Create the nav_patrol state machine using a Concurrence container
        self.detect_rotate=Concurrence(outcomes=['detect0','detect1','detect2','detect3','detect_other', 'not_find'],
                                        default_outcome='not_find',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        
        with self.detect_rotate:
            Concurrence.add('DETECT_TAG', DetectTag())
            Concurrence.add('ROTATING', Rotate())

        # Initialize the patrol state machine
        self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Add the states to the state machine with the appropriate transitions
        with self.sm_patrol:
            StateMachine.add('GO_TO_0', nav_states[0], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            StateMachine.add('GO_TO_1', nav_states[1], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'})
            StateMachine.add('GO_TO_2', nav_states[2], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            StateMachine.add('GO_TO_3', nav_states[3], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            
            StateMachine.add('GO_TO_detect_rotate', self.detect_rotate,transitions={'detect0':'GO_TO_1','detect1':'GO_TO_2','detect2':'GO_TO_3','detect3':'GO_TO_0', 'detect_other':'GO_TO_0', 'not_find':'GO_TO_detect_rotate'})
        
        # Create and start the SMACH introspection server

        intro_server = IntrospectionServer('patrol_in', self.sm_patrol, '/SM_ROOT')
        intro_server.start()

        # Set the shutdown function (stop the robot)
        sm_outcome = self.sm_patrol.execute()
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
        
        #rospy.on_shutdown(self.shutdown)
        intro_server.stop()

    def concurrence_child_termination_cb(self, outcome_map):
        if outcome_map['DETECT_TAG']=='detect0' or outcome_map['DETECT_TAG']=='detect1' or outcome_map['DETECT_TAG']=='detect2' or outcome_map['DETECT_TAG']=='detect3' or outcome_map['DETECT_TAG']=='detect_other':
            rospy.loginfo('preempted due to detection of tag')
            return True
        if outcome_map['ROTATING']=='full_rotate':
            rospy.loginfo('preempted in the due to full rotation')
            return True

        return False

    def concurrence_outcome_cb(self, outcome_map):
        if outcome_map['DETECT_TAG']=='detect0':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect0'

        elif outcome_map['DETECT_TAG']=='detect1':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect1'

        elif outcome_map['DETECT_TAG']=='detect2':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect2'

        elif outcome_map['DETECT_TAG']=='detect3':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect3'

        elif outcome_map['DETECT_TAG']=='detect_other':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect_other'

        elif outcome_map['ROTATING']=='full_rotate':
            rospy.loginfo('it is full_rotate in the concurrancy outcome')
            return 'not_find'


        else:
            return 'not_find'
            
        
    def image_callback(self, userdata, ros_image):
        
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        #rospy.loginfo('initiating...')
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
            
        except CvBridgeError, e:
            print e
            
        
        #rospy.loginfo('IMAGE RECEIVED')
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        display_image = self.process_image(frame)
                     
        # Display the image.
        cv2.imshow("Image_Window", display_image)
        
        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
       
        #rospy.loginfo('conc 1')
        rospy.sleep(10)
        return True
    
    
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
       # grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges
    
       
    def image_callback1(self, userdata, ros_image):
        
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
            rospy.loginfo('ERROR ERROR!!!')
        #rospy.loginfo('IMAGE RECEIVED')
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        display_image = self.process_image1(frame)
        #rospy.loginfo('image view window final')
               
        # Display the image.
        cv2.imshow("Image_Window1", display_image)
        
        # Process any keyboard commands
        #self.keystroke = cv.WaitKey(5)

        

    def process_image1(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        #edges = cv2.Canny(grey, 15.0, 30.0)
        
        return grey
    

    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.n_succeeded += 1
        elif status == actionlib.GoalStatus.ABORTED:
            self.n_aborted += 1
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.n_preempted += 1

        try:
            #rospy.loginfo("n_succeeded : " + str(self.n_succeeded))
            #rospy.loginfo("n_aborted : " + str(self.n_aborted))
            #rospy.loginfo("n_preempted : " + str(self.n_preempted))
            rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
        except:
            pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_patrol.request_preempt()        
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
