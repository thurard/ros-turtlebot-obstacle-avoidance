
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS python programming with finite state machines to describe a robot's behaviors
Vincent Hugel, Seatech/SYSMER 2A Course
Refactored and fixed.
"""

#############################################################################
# imports
#############################################################################
import rospy
import math
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist
from fsm import fsm
from kobuki_msgs.msg import BumperEvent

#############################################################################
# class RobotBehavior
#############################################################################
class RobotBehavior(object):
    #############################################################################
    # constructor, called at creation of instance
    #############################################################################
    def __init__(self, handle_pub, T):
        self.twist = Twist()
        self.twist_real = Twist()
        self.vreal = 0.0 # longitudinal velocity
        self.wreal = 0.0 # angular velocity
        self.vmax = 1.5
        self.wmax = 4.0
        self.previous_signal = 0
        self.button_pressed = False
        self.joy_activated = False
        self.pub = handle_pub
        self.T = T
        self.cpt = 0
        self.cpt1 = 0
        self.cpt2 = 0
        self.cpt3 = 0
        self.cpt4 = 0
        self.obsdetect = False
        
        self.fs = fsm([ 
            ("Start", "JoyControl", True),
            ("JoyControl", "AutonomousMode1", self.check_JoyControl_To_AutonomousMode1, self.DoAutonomousMode1),
            ("JoyControl", "JoyControl", self.KeepJoyControl, self.DoJoyControl),
            
            ("AutonomousMode1", "JoyControl", self.check_AutonomousMode1_To_JoyControl, self.DoJoyControl),
            ("AutonomousMode1", "Stop1", self.check_AutonomousMode1_To_Stop1, self.DoStartStop1),
            ("AutonomousMode1", "AutonomousMode1", self.KeepAutonomousMode1, self.DoAutonomousMode1),
            
            ("Stop1", "Recule", self.check_Stop1_To_Recule, self.DoStartRecule),
            ("Stop1", "Stop1", self.KeepStop1, self.DoKeepStop1),
            
            ("Recule", "Stop2", self.check_Recule_To_Stop2, self.DoStartStop2),
            ("Recule", "Recule", self.KeepRecule, self.DoKeepRecule),
            
            ("Stop2", "Rotate", self.check_Stop2_To_Rotate, self.DoStartRotate),
            ("Stop2", "Stop2", self.KeepStop2, self.DoKeepStop2),
            
            ("Rotate", "Stop3", self.check_Rotate_To_Stop3, self.DoStartStop3),
            ("Rotate", "Rotate", self.KeepRotate, self.DoKeepRotate),
            
            ("Stop3", "AutonomousMode1", self.check_Stop3_To_AutonomousMode1, self.DoAutonomousMode1),
            ("Stop3", "Stop3", self.KeepStop3, self.DoKeepStop3)
        ])

    #############################################################################
    # callback for joystick feedback
    #############################################################################
    def callback(self, data):
        self.twist.linear.x = self.vmax * data.axes[1]
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.wmax * data.axes[4]

        # for transition conditions of fsm
        if not self.button_pressed:
            self.button_pressed = (self.previous_signal == 0 and data.buttons[0] == 1)      
        self.previous_signal = data.buttons[0]

        self.joy_activated = (abs(data.axes[1]) > 0.05 or abs(data.axes[2]) > 0.05)
        
        
   
    def processScan(self, data):
        # Si un obstacle est déjà détecté, on ne fait rien pour économiser le calcul
        if self.obsdetect:
            return

        dist_detect = 0.5  # Distance de sécurité (0.5 mètres)
        
        # On regarde toutes les mesures du laser
        for value in data.ranges:
            # math.isnan vérifie que la valeur est un vrai nombre (pas une erreur)
            if not math.isnan(value) and value < dist_detect:
                rospy.loginfo("Obstacle Lidar detecte a : " + str(value))
                self.obsdetect = True
                break  # On arrête de chercher dès qu'on trouve un obstacle
        
        

     
        
        

    #############################################################################
    # smoothing velocity function
    #############################################################################
    def smooth_velocity(self):
        accmax = 0.01
        accwmax = 0.05
        vjoy = 0.0
        wjoy = 0.0
        vold = 0.0
        wold = 0.0  

        # filter twist
        vjoy = self.twist.linear.x
        vold = self.vreal
        deltav_max = accmax / self.T
    
        # vreal
        if abs(vjoy - self.vreal) < deltav_max:
            self.vreal = vjoy
        else:
            sign_ = 1.0
            if vjoy < self.vreal:
                sign_ = -1.0
            else:
                sign_ = 1.0
            self.vreal = vold + sign_ * deltav_max
    
        # saturation
        if self.vreal > self.vmax:
            self.vreal = self.vmax
        elif self.vreal < -self.vmax:
            self.vreal = -self.vmax     
    
        # filter twist
        wjoy = self.twist.angular.z
        wold = self.wreal
        deltaw_max = accwmax / self.T
    
        # wreal
        if abs(wjoy - self.wreal) < deltaw_max:
            self.wreal = wjoy
        else:
            sign_ = 1.0
            if wjoy < self.wreal:
                sign_ = -1.0
            else:
                sign_ = 1.0
            self.wreal = wold + sign_ * deltaw_max
        # saturation
        if self.wreal > self.wmax:
            self.wreal = self.wmax
        elif self.wreal < -self.wmax:
            self.wreal = -self.wmax     
            
        self.twist_real.linear.x = self.vreal   
        self.twist_real.angular.z = self.wreal
    
    #############################################################################
    # functions for fsm transitions
    #############################################################################

    def check_JoyControl_To_AutonomousMode1(self, fss):
        return self.button_pressed

    def check_AutonomousMode1_To_JoyControl(self, fss):
        return self.joy_activated
        
    def check_AutonomousMode1_To_Stop1(self, fss):
        if self.obsdetect == True:
            return True
        else:
            return False
    
    def check_Stop1_To_Recule(self, fss):
        seuil = 10
        if self.cpt > seuil:
            return True
        else:
            return False
            
    def KeepStop1(self, fss):
        return not self.check_Stop1_To_Recule(fss)
    
    def DoKeepStop1(self, fss, value):
        self.cpt = self.cpt + 1
        pass
        
    def check_Recule_To_Stop2(self, fss):
        seuil = 10
        if self.cpt1 > seuil:
            return True
        else:
            return False
            
    def KeepRecule(self, fss):
        return not self.check_Recule_To_Stop2(fss)
        
    def DoKeepRecule(self, fss, value):
        self.cpt1 = self.cpt1 + 1
    
    def check_Stop2_To_Rotate(self, fss):
        seuil = 10
        if self.cpt2 > seuil:
            return True
        else:
            return False
            
    def KeepStop2(self, fss):
        return not self.check_Stop2_To_Rotate(fss)
    
    def DoKeepStop2(self, fss, value):
        self.cpt2 = self.cpt2 + 1
        
    def check_Rotate_To_Stop3(self, fss):
        seuil = 10
        if self.cpt3 > seuil:
            return True
        else:
            return False
            
    def KeepRotate(self, fss):
        return not self.check_Rotate_To_Stop3(fss)
    
    def DoKeepRotate(self, fss, value):
        self.cpt3 = self.cpt3 + 1
    
    def DoKeepStop3(self, fss, value):
        self.cpt4 = self.cpt4 + 1
    
    def check_Stop3_To_AutonomousMode1(self, fss):
        seuil = 10
        if self.cpt4 > seuil:
            return True
        else:
            return False
            
    def KeepStop3(self, fss):
        return not self.check_Stop3_To_AutonomousMode1(fss)

    def KeepJoyControl(self, fss):
        return not self.check_JoyControl_To_AutonomousMode1(fss)

    def KeepAutonomousMode1(self, fss):
        return not (self.check_AutonomousMode1_To_JoyControl(fss) or self.check_AutonomousMode1_To_Stop1(fss))

    #############################################################################
    # functions for instructions inside states of fsm
    #############################################################################
    def DoJoyControl(self, fss, value):
        self.button_pressed = False
        self.smooth_velocity()
        self.pub.publish(self.twist_real)
        pass

    def DoAutonomousMode1(self, fss, value):
        self.obsdetect = False
        self.cpt4 = 0
        self.button_pressed = False
        # go forward
        go_fwd = Twist()
        go_fwd.linear.x = self.vmax / 3.0
        self.pub.publish(go_fwd)
        pass

    def DoStartStop1(self, fss, value):
        self.cpt = 0
        self.obsdetect = False 
        go_fwd = Twist()
        go_fwd.linear.x = 0
        self.pub.publish(go_fwd)
        pass

    def DoStartRecule(self, fss, value):
        self.cpt = 0
        self.cpt1 = 0
        go_fwd = Twist()
        go_fwd.linear.x = -self.vmax / 3.0
        self.pub.publish(go_fwd)
        pass

    def DoStartStop2(self, fss, value):
        self.cpt1 = 0
        self.cpt2 = 0
        go_fwd = Twist()
        go_fwd.linear.x = 0
        self.pub.publish(go_fwd)
        pass

    def DoStartRotate(self, fss, value):
        self.cpt2 = 0
        self.cpt3 = 0
        go_fwd = Twist()
        go_fwd.angular.z = self.wmax / 2.0
        self.pub.publish(go_fwd)
        pass

    def DoStartStop3(self, fss, value):
        self.cpt3 = 0
        self.cpt4 = 0
        go_fwd = Twist()
        go_fwd.angular.z = 0
        self.pub.publish(go_fwd)
        pass

    def processBump(self, data):
        rospy.loginfo("Collision %d", data.bumper)
        if self.obsdetect == False:
            if data.state == BumperEvent.PRESSED:
                self.obsdetect = True

#############################################################################
# main function
#############################################################################
if __name__ == '__main__':
    try:
        rospy.init_node('joy4ctrl')
        # real turtlebot2
        pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        
        Hz = 10
        rate = rospy.Rate(Hz)
        T = 1.0 / Hz

        MyRobot = RobotBehavior(pub, T)
        rospy.Subscriber("joy", Joy, MyRobot.callback)
        MyRobot.fs.start("Start")

        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, MyRobot.processBump)
        #Lidar
        rospy.Subscriber("/scan", LaserScan, MyRobot.processScan)

        # loop at rate Hz
        while not rospy.is_shutdown():
            ret = MyRobot.fs.event("")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass