'''
! /usr/bin/env python3

Author: Amit Kumar Singh
email: ai22mtech12010@iith.ac.in
'''
import rospy
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from oops import ControlTheDroneClass

 

if __name__ == '__main__':
    
    targetAltitude = 1.0
    rospy.init_node("actuatingTheDrone", anonymous = True)
    obj = ControlTheDroneClass()
    obj.arm_and_takeoff_nogps(targetAltitude)

    try:
        obj.ControllerFunction()
        rospy.spin()
    except rospy.ROSInterruptException():
        rospy.loginfo("Node terminated.")
