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

pub = rospy.Publisher('/auto_cmd_vel', Twist, queue_size = 1)
move_cmd = Twist()
cpt = 0
range_max = 0.
cap_consigne = 0.
max_speed = 1.0
speed = 0.0
omega = 0.0
larg_couloir = 0.3

speed_max  = 0.5
omega_max = 2.5

droite_gauche = True


def callback_odometry_seul(odometry):
    print("Inside Callback Odom_seul: ")
    
def callback_command_seul(command):
    #print("Inside Callback Command_seul: ")
    global speed
    global omega
    speed = command.linear.x
    omega = command.angular.z
    
def callback_laser_seul(laser):
    print("Inside Callback Laser_seul: ")

def callback_odom_command(odometry, command):
    print("Inside Callback Odometry Command: ")
    
def callback_command(laser, command):
    print("Inside Callback Command: ")
    
def callback_laser(laser, odometry):
    #print("Inside Callback Laser: ")
    global cpt
    global range_max
    global cap_consigne
    global speed
    global omega
    global larg_couloir
    global omega_max
    global speed_max
    
    #  code a mettre
    
    #print(laser.ranges[0])  # affichage de la premiere info du capteur
    
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
        dref = 0.5
    else:
        x1 = dA * sin(-1.5)
        x2 = dB * sin(-1.3)
        
        y1 = dA * cos(-1.5)
        y2 = dB * cos(-1.3)
        
        theta = math.atan( (x2-x1) / (y2-y1))
        
        d=-(dA+dB)/2           
        dref =- 0.5
        
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
    
    Kd=5
    u1 = 0.3
    u2 = - u1/ (0.2*cos(theta))*sin(theta) - u1/cos(theta)* Kd*(d-dref)
        
    move_cmd.linear.x = u1
    move_cmd.angular.z = u2
    
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
    
    rospy.init_node('SuiviMur', anonymous=True)
    
    speed_max = rospy.get_param('~speed_max', 0.5)
    omega_max = rospy.get_param('~omega_max', 2.5)
    
    laser_sub = message_filters.Subscriber("/simple_scan", LaserScan)
    #joy_sub = rospy.Subscriber("/joy", Joy, callbackjoy)
    odometry_sub = message_filters.Subscriber("/odom", Odometry)
    command_sub = message_filters.Subscriber("/manu_cmd_vel",Twist)     # a enlever ?
    
    range_max = rospy.get_param('~range_max', 5.0)
    cap_consigne = rospy.get_param('~cap_consigne', 0.0)     # a enlever ?
    max_speed = rospy.get_param('~max_speed', 0.2)
    larg_couloir = rospy.get_param('~largeur_couloir', 0.3)    # a enlever ?
    
    rospy.loginfo('Parameter range_max : %f m \n\t cap_consigne %s rd \n\t max_speed %f m/s', range_max, cap_consigne, max_speed)
    
    ts = message_filters.ApproximateTimeSynchronizer([laser_sub, odometry_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_laser)
       
    ts = message_filters.ApproximateTimeSynchronizer([command_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback_command_seul)

    print("C'est parti... ")
    rospy.spin()

