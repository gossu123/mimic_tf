#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from std_msgs.msg import String
from turtlesim.msg import Pose
roslib.load_manifest('turtle_tf1')


class turtlePose():
    pose = Pose()
    
    
    def updating_pose(self, data):
        """Gets pose data, received by the subscriber"""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        
        

if __name__ == '__main__':


    rospy.init_node('listener')
    listener = tf.TransformListener()


    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    # to get pose of turtle2
    pose_data = turtlePose()
    rospy.Subscriber('/turtle2/pose', Pose, pose_data.updating_pose)
    
    
    rate = rospy.Rate(10.0)
    
    i = 0
    
    while not rospy.is_shutdown():
        try:
        
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            if i == 0:
                distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                i = i + 1
                
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            continue

        theta = tf.transformations.euler_from_quaternion(rot)[2]
        
        
        distance1 = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        
        cmd = geometry_msgs.msg.Twist()
        if (distance1 - distance < -0.05 and trans[0] >= 0):
            linear = -1 * distance
        elif (distance1 - distance > 0.05 and trans[0] >= 0):
            linear = distance
        elif (distance1 - distance < -0.05 and trans[0] < 0):
            linear = distance
        elif (distance1 - distance > 0.05 and trans[0] < 0):
            linear = -1 * distance
        else:
            linear = 0
        
        if theta < -0.05:
            time = abs(theta) * 0.2
            cmd.angular.z = -1 * abs(theta) * 5
            rospy.sleep(time)
        elif theta > 0.05:
            time = abs(theta) * 0.2
            cmd.angular.z = abs(theta) * 5
            rospy.sleep(time)
        elif theta < -1:
            time = abs(theta) * 0.2
            cmd.angular.z = -1 * abs(theta) * 10
        elif theta > 1:
            time = abs(theta) * 0.2
            cmd.angular.z = abs(theta) * 10
            rospy.sleep(time)
        else:
            cmd.angular.z = 0
        
        cmd.linear.x = linear
        turtle_vel.publish(cmd)
