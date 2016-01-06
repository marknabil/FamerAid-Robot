class DetectTag(smach.State):
    def __init__(self):
        rospy.loginfo('Initilizing DetectTag')
        smach.State.__init__(self, outcomes=['detect0','detect1','detect2','detect3','detect_other', 'not_find'])
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.tagDetected)
        self.tag_ids = [6,5,3,2]

        self.my_tag = 999
        self.number_of_marker = 0
        self.flag=False
        #self.positionX = 0

    def execute(self, userdata):
        rospy.loginfo('Executing DetectTag Class...')
        self.flag = True
        
        #and self.positionX < d
        while(self.number_of_marker==0 ):

            if(self.preempt_requested()):
                rospy.loginfo("state DetectTag is being preempted!!")
                self.service_preempt()

                self.my_tag = 999
                self.number_of_marker=0
                self.flag = False
                return 'not_find'

            rospy.sleep(1)

     
        # and self.positionX < d
        if(self.my_tag==2 ):
            rospy.loginfo('detected 2' )
            self.my_tag = 999
            self.flag = False
            self.number_of_marker=0
            return 'detect0'

        elif (self.my_tag==3 ):
            rospy.loginfo('detected 3')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect1'

        elif (self.my_tag==5 ):
            rospy.loginfo('detected 5')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect2'

        elif (self.my_tag==6 ):
            rospy.loginfo('detected 7')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect3'

        else :
            rospy.loginfo( 'other tag than the pre defined')
            self.number_of_marker=0
            self.flag = False
            return 'detect_other'
            

    def tagDetected(self, msg):
        #rospy.loginfo('trying to detect')
        if self.flag:

            # Get the number of markers
            self.number_of_marker = len(msg.markers)
            
            # If no markers detected, just return
            if self.number_of_marker == 0:
                return

            # Iterate through the tags and sum the x, y and z coordinates            
            for tag in msg.markers:
                
                # Skip any tags that are not in our list
                if self.tag_ids is not None and not tag.id in self.tag_ids:
                    continue

                rospy.loginfo('detect something')
                self.my_tag = tag.id
                self.positionX=tag.pose.pose.position.x
                #rospy.loginfo('detected tag: %d', tag.id,'and distance is %d',tag.pose.pose.position.x)
