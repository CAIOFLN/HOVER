#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3



def control():
    rospy.init_node('controle_robo', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        cmd_vel_msg = Twist()
        a = input()
        if a =="w":
            cmd_vel_msg.linear.x = 0.2  # Velocidade linear (m/s)
        elif a == "s":
            cmd_vel_msg.linear.x = -0.2  # Velocidade linear (m/s)
        elif a == "d":
            cmd_vel_msg.angular.z = 0.5  # Velocidade angular (rad/s)
        elif a == "a":
            cmd_vel_msg.angular.z = -0.5  # Velocidade angular (rad/s)
        else:
            cmd_vel_msg.angular.z = 0  # Velocidade angular (rad/s)
            cmd_vel_msg.linear.x = 0  # Velocidade angular (rad/s)
        cmd_vel_pub.publish(cmd_vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
