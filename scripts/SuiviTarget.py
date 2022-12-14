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
cpt = 0
range_max = 0.
cap_consigne = 0.
max_speed = 1.0
speed = 0.0
omega = 0.0
larg_couloir = 0.3

droite_gauche = False
current = 0



def callback_laser(laser, odometry , markers):
    #print("Inside Callback Laser: ")
    global cpt
    global range_max
    global cap_consigne
    global speed
    global omega
    global larg_couloir
    global current
    #  code a mettre
    
    
    
    # moyennage du laser 147
    dA =  laser.ranges[15]
    dB = laser.ranges[14]

    dC = (laser.ranges[7] + laser.ranges[8])/2

    dD = laser.ranges[1]
    dE = laser.ranges[0]

        
       
    # calcul de l'angle et de la distance
    
    if droite_gauche:
        x1 = dE * sin(1.5)
        x2 = dD * sin(1.3)
        
        y1 = dE * cos(1.5)
        y2 = dD * cos(1.3)
        
        theta = math.atan( (x2-x1) / (y2-y1))
        
        d= (dD+dE)/2    
        dref = 0.3
    else:
        x1 = dA * sin(-1.5)
        x2 = dB * sin(-1.3)
        
        y1 = dA * cos(-1.5)
        y2 = dB * cos(-1.3)
        
        theta = math.atan( (x2-x1) / (y2-y1))
        
        d=-(dA+dB)/2           
        dref =- 0.3
        
    #print(theta)
    #print(d)

    #quaternion=(
        #odometry.pose.pose.orientation.x,
        #odometry.pose.pose.orientation.y,
        #odometry.pose.pose.orientation.z,
        #odometry.pose.pose.orientation.w)
    #euler=tf.transformations.euler_from_quaternion(quaternion)
    #yaw=euler[2]
        
    
    
    # Premiere version suit l'angle du mur
    #move_cmd.linear.x = 0.1
    #move_cmd.angular.z = - theta
    
    # dexieme version garde une distance constante au mur
    
    #Kd=5
    #u1 = 0.5
    #u2 = - u1/ (0.2*cos(theta))*sin(theta) - u1/cos(theta)* Kd*(d-dref)
    dist = -1
    for i in range(len(markers.markers)):
        if markers.markers[i].id==current:
            dist =  markers.markers[i].pose.pose.position.z
            cote = markers.markers[i].pose.pose.position.x
            
            if dist < 1:
                current = current +1
                print ("Now following ",  current)
    
    if current > 8:
        current = 0
    
    # Premiere version suit l'angle du mur
    #move_cmd.linear.x = 0.1
    #move_cmd.angular.z = - theta
    
    # dexieme version garde une distance constante au mur
    #dref = 0.3
    #Kd=5
    u1=0
    u2 = 0
    #print(dist)
    if dist > 0.5:
        u1 = 0.3
        u2 = -cote    
        
    move_cmd.linear.x = u1
    move_cmd.angular.z = u2
    #print(move_cmd)
        
    pub.publish(move_cmd)
    
    

if __name__ == '__main__':
    
    rospy.init_node('command_by_laser', anonymous=True)
    laser_sub = message_filters.Subscriber("/simple_scan", LaserScan)
    #joy_sub = rospy.Subscriber("/joy", Joy, callbackjoy)
    odometry_sub = message_filters.Subscriber("/odom", Odometry)
    command_sub = message_filters.Subscriber("/manu_cmd_vel",Twist)     # a enlever ?
    aruco_sub = message_filters.Subscriber("/aruco_marker_publisher/markers",MarkerArray)    
    
    range_max = rospy.get_param('~range_max', 5.0)
    cap_consigne = rospy.get_param('~cap_consigne', 0.0)     # a enlever ?
    max_speed = rospy.get_param('~max_speed', 0.2)
    larg_couloir = rospy.get_param('~largeur_couloir', 0.3)    # a enlever ?
    
    rospy.loginfo('Parameter range_max : %f m \n\t cap_consigne %s rd \n\t max_speed %f m/s', range_max, cap_consigne, max_speed)
    
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odometry_sub,aruco_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_laser)
       
    ts = message_filters.ApproximateTimeSynchronizer([command_sub], 10, 0.1, allow_headerless=True)

    print("C'est parti... ")
    rospy.spin()

