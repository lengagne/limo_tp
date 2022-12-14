#! /usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan


Simulation = True

pub = rospy.Publisher('/simple_scan', LaserScan, queue_size = 1)

SimpleLaser = LaserScan()
SimpleLaser.header.frame_id = "laser_link"
SimpleLaser.angle_min = -1.6
SimpleLaser.angle_max = 1.6
SimpleLaser.angle_increment = 0.2
SimpleLaser.range_min = 0.1
SimpleLaser.range_max = 100.0

nb = int((SimpleLaser.angle_max - SimpleLaser.angle_min) / SimpleLaser.angle_increment)
for j in range(nb):
     SimpleLaser.ranges.append(0)
     SimpleLaser.intensities.append(42)


def simplify_laser( laser):
    global pub
    SimpleLaser.header = laser.header
    j = 0
    SimpleLaser.ranges[j] = 0
    cptj=0
    maxJ = int((SimpleLaser.angle_max - SimpleLaser.angle_min) / (SimpleLaser.angle_increment))
    
    for i in range( len(laser.ranges)):
        current_angle = laser.angle_min + i * laser.angle_increment        
        min_angle = SimpleLaser.angle_min + (j) * SimpleLaser.angle_increment
        max_angle = SimpleLaser.angle_min + (j+1) * SimpleLaser.angle_increment
        if ( current_angle > max_angle ):
            j = j+1
            if j >= maxJ:
                break
            
            SimpleLaser.ranges[j] = 0
            cptj = 0

        if (laser.intensities[i] != 0 or Simulation):
            if ( current_angle >= min_angle and current_angle <= max_angle):
                SimpleLaser.ranges[j] = (SimpleLaser.ranges[j] * cptj + laser.ranges[i])/(cptj+1)
                cptj = cptj+1
                
    for i in range( len(SimpleLaser.ranges)):
        if SimpleLaser.ranges[i] > SimpleLaser.range_max:
            SimpleLaser.ranges[i] = SimpleLaser.range_max
                
    pub.publish(SimpleLaser)
    

if __name__ == '__main__':
    
    #Simulation = rospy.get_param('simulation', false)
    
    
    rospy.init_node('laser_simplifier', anonymous=True)
    
    scan_sub = rospy.Subscriber("/scan", LaserScan, simplify_laser )

    rospy.spin()

