class Rotate(State):
    def __init__(self):
        State.__init__(self, outcomes=['full_rotate','not_full_rotate'])

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist)

        self.r = rospy.Rate(20)
        self.angular_speed = 0.6
        self.angular_tolerance = radians(2.5)
        self.goal_angle = 2*pi

    def execute(self, userdata):
        rospy.loginfo("Executing Rotate Class...")

        # Get the starting position values     
        (position, rotation) = self.get_odom()
        last_angle = rotation
        turn_angle = 0

        rotate_command = Twist()
        rotate_command.angular.z = self.angular_speed
        #and not userdata.detectTag_Flag
        while abs(turn_angle + self.angular_tolerance) < abs(self.goal_angle) and not rospy.is_shutdown() :

            if self.preempt_requested():
                rospy.loginfo("State Rotate is being preempted!!!")
                self.service_preempt()
               
                return 'not_full_rotate'

            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(rotate_command)
            self.r.sleep()
                
            # Get the current rotation
            (position, rotation) = self.get_odom()
                
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
                
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
                
        # Stop the robot before the next leg
        rotate_command = Twist()
        self.cmd_vel.publish(rotate_command)
        rospy.sleep(1)

        return 'full_rotate'

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
