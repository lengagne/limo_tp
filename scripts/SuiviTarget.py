#! /usr/bin/env python

import math
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
#from jaguar.msg import OdometryNative
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
#import geometry_msgs
from numpy import pi, arange, sin, cos, sqrt, arcsin, arctan2, linspace
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from aruco_msgs.msg import MarkerArray

pub = rospy.Publisher('/auto_cmd_vel', Twist, queue_size = 1)
move_cmd = Twist()

speed_max = 0.5
omega_max = 2.5
current_marker = 0



def callback_laser(laser, odometry , markers):
    print("Inside Callback Laser,Odometry,Marker: ")
    global speed_max
    global omega_max
    global current_marker

    #  code a mettre
    
    # localisation du mur  a partir du laser
    # laser.ranges[0] correspond a la distance suivant un angle de -1.5 radian
    # laser.ranges[1] correspond a la distance suivant un angle de -1.3 radian
    # ...
    # laser.ranges[14] correspond a la distance suivant un angle de 1.3 radian
    # laser.ranges[15] correspond a la distance suivant un angle de 1.5 radian

    # distance et orientation par rapport au mur
    theta = 0
    d = 0
    print(theta)
    print(d)    
    
    dist = -1
    for i in range(len(markers.markers)):
        if markers.markers[i].id==current_marker:
            # on recupere la pose du marqueur courant
            # markers.markers[i].pose.pose.
            
            # on suppose la distance au markeur courant comme la distance suivant l'axe z
            dist =  markers.markers[i].pose.pose.position.z
            
            # on suppose le decalage au markeur courant comme la distance suivant l'axe x
            cote = markers.markers[i].pose.pose.position.x
            
            # si on est a moins de 1 metre du marqueur courant on change de marqueur
            if dist < 1:
                current_marker = current_marker +1
                print ("Now following ",  current_marker)
    
    # si on depasse le marqueur 8 on recommence a 0
    if current_marker > 8:
        current_marker = 0


    # vitesse lineaire et angulaire du robot
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = - theta

    # on s'assure qu'on ne depasse pas les vitesses max
    if move_cmd.linear.x > speed_max:
        move_cmd.linear.x = speed_max
    if move_cmd.linear.x < -speed_max:
        move_cmd.linear.x = -speed_max

    if move_cmd.angular.z > omega_max:
        move_cmd.angular.z = omega_max
    if move_cmd.angular.z < -omega_max:
        move_cmd.angular.z = -omega_max
        
    pub.publish(move_cmd)
    
    

if __name__ == '__main__':
    
    rospy.init_node('command_by_laser', anonymous=True)
    
    speed_max = rospy.get_param('~speed_max', 0.5)
    omega_max = rospy.get_param('~omega_max', 2.5)

    
    laser_sub = message_filters.Subscriber("/simple_scan", LaserScan)
    odometry_sub = message_filters.Subscriber("/odom", Odometry)
    aruco_sub = message_filters.Subscriber("/aruco_marker_publisher/markers",MarkerArray)    
    command_sub = message_filters.Subscriber("/manu_cmd_vel",Twist)
    
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odometry_sub,aruco_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_laser)
       
    ts = message_filters.ApproximateTimeSynchronizer([command_sub], 10, 0.1, allow_headerless=True)

    print("C'est parti... ")
    rospy.spin()

