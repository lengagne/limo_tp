#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerRequest


twist = Twist()
dist=0.0
A_button=False
B_button=False
speed_max=0.0
omega_max=0.0
mode="reel"

def  update_joystick(msg):
    global A_button
    global B_button
    global speed_max
    global omega_max
    global mode
    if(mode=="reel"):
        twist.linear.x = msg.axes[1]*speed_max
    elif(mode=="simu"):
        twist.linear.x = abs(msg.axes[1])*speed_max #valeur absolue sinon on plante Gazebo : le robot fait toujours en avancant
    else:
        twist.linear.x = 0.0
    #twist.angular.z = msg.axes[0]*omega_max;
    twist.angular.z = (-msg.buttons[1] + msg.buttons[3])*omega_max;
    if(msg.buttons[4]==1 or msg.buttons[5]==1  or msg.buttons[6]==1  or msg.buttons[7]==1):
        A_button=True
    if(msg.buttons[9]==1):
        B_button=True        
    
def update_distance(msg):
    global dist;
    dist_max = 1000;
    '''
    for i in range(0, 450):
        if (msg.intensities[i] > 450): 
            if(msg.ranges[i] < dist_max):
                dist_max = msg.ranges[i]
    '''
    dist = dist_max

if __name__ == '__main__':

    rospy.init_node('control_robot')
    speed_max = rospy.get_param('~speed_max', "0.5")
    omega_max = rospy.get_param('~omega_max', "0.5")
    cmd_vel_pub_name = rospy.get_param('~cmd_vel_pub_name', "/manu_cmd_vel")
    mode = rospy.get_param('~mode', "reel")
    rospy.Subscriber("joy",Joy,update_joystick,queue_size =10)
    rospy.Subscriber("scan",LaserScan,update_distance,queue_size =10)
    pub = rospy.Publisher(cmd_vel_pub_name, Twist,queue_size=10)    

    rospy.wait_for_service('/limo_manu_mode')
    rospy.wait_for_service('/limo_auto_mode')

    Abutton_service = rospy.ServiceProxy('/limo_manu_mode',Trigger)
    Bbutton_service = rospy.ServiceProxy('/limo_auto_mode',Trigger)

    auto = TriggerRequest()
    manu = TriggerRequest()

    rate = rospy.Rate(5)

    A_button = False
    B_button = False

    while not rospy.is_shutdown():
        if(A_button):
            A_button=False
            response = Abutton_service(manu)
        if(B_button):
            B_button=False
            response = Bbutton_service(auto)
        
        desired_twist = twist
        pub.publish(desired_twist)
        rate.sleep()
