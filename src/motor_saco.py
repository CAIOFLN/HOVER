#!/usr/bin/env python3
from motores import ESC
import rospy
from std_srvs.srv import Empty, EmptyResponse

class EscControl(object):
    
    def __init__(self)-> None:
        self.serv_hover = rospy.Service('/hover', Empty, self.hover)
        
        self.esc = ESC(pin1=13)
        self.off = True
        
    def hover(self, req:Empty)-> EmptyResponse:
        self.off = not self.off
        
        if not self.off:
            self.esc.calibrate()
            self.esc.arm()
            
        else:
            self.esc.halt()
            
        return EmptyResponse()