#!/usr/bin/env python3

import rospy
from raspcamerapub import Camera 
from controle_robo import SET_POINT
from line_follower import Controle
from motor_saco import EscControl
from motor_bunda import ServoControl



if __name__ == '__main__':
    rospy.init_node('hover')
    
    setpoint = SET_POINT()
    controle = Controle()
    camera = Camera()
    bunda = ServoControl()
    saco = EscControl()
    rate = rospy.Rate(5) # 5 Hz
    while not rospy.is_shutdown():
        pass
        camera.publish_image()
        rate.sleep()
    
    camera.close_camera()
