#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Transform
from TrinaPointAndClick.msg import Marker, MarkerArray

class Marker_Node():
    """
    This node listens to marker tracker data and keeps track of the pose of all markers seen during run-time. This node also computes the transform from the robot frame to the marker.
    """
    def __init__(self):
        """
        Initialise the list of markers and the subscribers and publishers.
        Parameters:
            None
        Returns:
            None
        """
        rospy.loginfo("Marker_node started.")

        #initialize list of markers
        self.marker_list = []
        
        #set up publishers and subscribers
        self.sub_marker_tracker = rospy.Subscriber('/MarkerPose', PoseStamped, self.callback_tracker)
        self.pub_marker_list = rospy.Publisher('/MarkerArray', MarkerArray, latch=True, queue_size=1)

        #create a ROS Timer for publishing data
        rospy.Timer(rospy.Duration(1.0/60.0), self.callback_publish_list)

        #keep the node alive while it is waiting for data
        rospy.loginfo("rospy.spin()")
        rospy.spin()
        
    def callback_tracker(self, data):
        """
        Takes a pose from the camera to a marker and adds or updates its values in the list.
        Parameters:
            data - containing a stamped pose from the camera to a marker as well as the marker id in the header
        Returns:
            None
        """
        
        #convert incoming stamped pose data to Marker message
        initial_data = self.convert_transform(data)
        
        #check if marker is not in the known marker set
        if initial_data.id_number <= 8 or initial_data.id_number == 999:
        
           #loops through the list to see if the marker id provided is in the list
            for i in xrange(len(self.marker_list)):
                
                #compare provided number with the i'th marker's ID number
                if self.marker_list[i].id_number == initial_data.id_number:
                    self.marker_list[i].workspace = initial_data.workspace
                    self.marker_list[i].visible = initial_data.visible
                    self.marker_list[i].time = initial_data.time
                    self.marker_list[i].transform = initial_data.transform
                    self.marker_list[i].distance = initial_data.distance     
                    
                    #return if the marker is found in the list
                    return None
                    
            #If marker is not found in the list, add marker to list
            self.marker_list.append(initial_data)
            
            #sorts the list by the marker id
            self.marker_list=sorted(self.marker_list, key=lambda marker: marker.id_number, reverse=True)

            
        else:
            return None

    def calculate_transform(self, data):
        """
        Takes a marker pose ROS message and returns a robot-base-frame-to-marker transform ROS message.
        Parameters:
            data - pose ROS message of marker relative to camera
        Returns:
            Transform ROS message of robot base frame to marker
        """


        #calculate the transform
        in_x = data.position.x
        in_y = data.position.y
        in_z = data.position.z
        
        input_translation = [in_x, in_y, in_z]
        multiplier = np.array([[ -0.02025737, -0.31392,  0.04627322],
                               [-0.38235706,  0.04113464, 0.03979437],
                               [-0.03673691, -0.27182984, -0.36413172 ]], dtype=np.float)
        offset = np.array([0.45368236, -0.14424458, 0.8933589], dtype=np.float)
        output_translation = np.matmul(multiplier, input_translation)+ offset

        #build the transform
        output_transform = Transform()
        output_transform.translation.x = output_translation[0]
        output_transform.translation.y = output_translation[1]
        output_transform.translation.z = output_translation[2]  
        #TODO: Check that the rotation transform is correct.
        output_transform.rotation = data.orientation
        
        return output_transform

    def calculate_distance(self, data):
        """
        Takes a transform and calculates the Euclidean distance to the marker from the robot base frame.
        Parameters:
            data - Transform ROS message of robot base frame to marker
        Returns:
            Euclidean distance to the marker from the robot base frame
        """
        #read in the data
        trans_x = data.translation.x
        trans_y = data.translation.y
        trans_z = data.translation.z

        #calculate the Euclidean distance
        distance = math.sqrt((0-trans_x)**2+(0-trans_y)**2+(0-trans_z)**2)

        return distance

    def convert_transform(self, data):
        """
        Takes a header and transform ROS message and converts it to a Marker message. The transform from the camera to the marker is modified to be the transform from the robot base frame to the marker.
        Parameters:
            data - Transform ROS message with header containing marker id
        Returns:
            Marker ROS message
        """

        #build the Marker message
        marker_message = Marker()
        marker_message.header.stamp = data.header.stamp
        marker_message.header.frame_id = "camera"
        marker_message.child_frame_id = "marker " + data.header.frame_id
        marker_message.id_number = int(data.header.frame_id)
        marker_message.workspace = self.classify_list(marker_message.id_number)
        marker_message.visible = True
        marker_message.time = rospy.get_time()
        marker_message.transform = self.calculate_transform(data.pose)
        marker_message.distance = self.calculate_distance(marker_message.transform)

        return marker_message

    def check_visible(self, duration):
        """
        Checks the list to see what markers should be marked as having not been visible for at given duration.
        Paramerters:
            duration - maximum time that a marker can not be detected before it is marked as not visible
        Returns:
            None
        """
        #current time
        now = rospy.get_time()
        #loop through list to find markers that are not visible
        for i in xrange(len(self.marker_list)):
            #last seen time
            last_seen = self.marker_list[i].time
            #check to see difference between time now and last time the marker was seen
            if (now - last_seen > duration):
                self.marker_list[i].visible = False

    def classify_list(self, marker_id):
        """
        Labels the provided marker if it is in the list of marker id numbers that are known to be the workspace markers.
        Parameters:
            marker_id - a Marker id number
        Returns:
            Marker ROS message
        """
        #hard coded list of workspace marker id numbers:
        list_of_marker_id = [0,1,2]
        #loops through list of workspace marker id numbers to check if the given number is in the list
        for i in xrange(len(list_of_marker_id)):
            if marker_id == list_of_marker_id[i]:
                return True
        return False
       
    def callback_publish_list(self, timer):
        """
        Publishes the list at a fixed rate (60Hz), which is the frame rate of the camera.
        
        Parameters:
            None
        Returns:
            None
        """
        #check list for markers that are no longer visible
        self.check_visible(0.5)

        #build the message to be published
        pub_list = MarkerArray()
        pub_list.header.stamp = rospy.Time.now()
        pub_list.header.frame_id = "base"
        pub_list.markers = self.marker_list
       
        #publish the marker list
        self.pub_marker_list.publish(pub_list)    
                          
if __name__ == '__main__':
    """
    Initializes node and names it
    
    Parameters:
        None
    Returns:
        None
    """
    print "Initializing Marker_Node..."
    
    rospy.init_node('Marker_Node')
    try: 
        Marker_Node = Marker_Node()
    except rospy.ROSInterruptException:
        rospy.logerror("Failed to start server node.")
        pass
