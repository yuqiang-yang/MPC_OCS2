#!/usr/bin/python3.8
import rospy
import time
import rtde_control
import rtde_receive
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from math import cos, acos, sin, asin, sqrt, exp, atan, atan2, pi, tan, ceil
import sys
import numpy as np
car_sub_record = []
ur_sub_record = []
is_new_control_msg = False
base_desired_vel = np.zeros(2)
arm_desired_vel = np.zeros(6)

#car state subsciber callback function
def car_sub_callback(odom:Odometry):
    global car_sub_record
    x = odom.pose.pose.orientation.x
    y = odom.pose.pose.orientation.y
    z = odom.pose.pose.orientation.z
    w = odom.pose.pose.orientation.w
    theta = atan2(2*(w*z+x*y),1-2*(y**2+z**2))
    car_sub_record = [odom.pose.pose.position.x, odom.pose.pose.position.y, theta ,odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z]

def wholebodycontrolcallback(msg:Float64MultiArray):
    global is_new_control_msg
    global arm_desired_vel
    global base_desired_vel
    is_new_control_msg = True
    base_desired_vel = msg.data[:2]
    arm_desired_vel = msg.data[2:]
    pass
class WBController():
    def __init__(self,base_pub,rtde_c,config:dict) -> None:
        self.base_pub = base_pub
        self.rtde_c = rtde_c

        if not config:
            self.car_linear_acc = config.car_linear_acc
            self.car_angular_acc = config.car_angular_acc
            self.car_linear_vel = config.car_linear_vel
            self.car_angular_vel = config.car_angular_vel
        else:
            self.car_linear_acc = 9999
            self.car_angular_acc = 9999
            self.car_linear_vel = 9999
            self.car_angular_vel = 9999
    def control(base_vel:np.array,joint_vel:np.array):


        pass

if __name__ == '__main__':
    rospy.init_node('mue_interface_node', anonymous=True)
    ur_ip = rospy.get_param('ur_ip','192.168.100.2')
    ur_control_activate = rospy.get_param('ur_control_activate',False)
    base_pub = rospy.Publisher ("/mobile_base/cmd_vel", Twist,queue_size=0) 
    base_sub = rospy.Subscriber("/mobile_base/odom",Odometry,car_sub_callback)
    
    wb_control_sun =  rospy.Subscriber("/wholebodycontrol",Float64MultiArray,wholebodycontrolcallback)
    velocity_and_acc_limit = rospy.get_param('velocity_and_acc_limit',True)
    if velocity_and_acc_limit:
        WBControllerConfig = dict()
        WBControllerConfig['car_linear_acc'] = rospy.get_param('car_linear_acc',0.5)
        WBControllerConfig['car_angular_acc'] = rospy.get_param('car_angular_acc',0.5)
        WBControllerConfig['car_linear_vel'] = rospy.get_param('car_linear_vel',0.5)
        WBControllerConfig['car_angular_vel'] = rospy.get_param('car_angular_vel',0.5)
    else:
        WBControllerConfig = dict()
    if ur_control_activate:
        for i in range(3):
            try:
                rtde_c = rtde_control.RTDEControlInterface(ur_ip)
                break
            except Exception:
                time.sleep(3)
                rospy.INFO('keep trying to connect RTDE Control')
                if i == 2:
                    sys.exit()    
    else:
        rtde_c = None
    rtde_r = rtde_receive.RTDEReceiveInterface(ur_ip)
    
    wbc = WBController(base_pub,rtde_c,WBControllerConfig)
    time.sleep(0.1)
    while base_sub.get_num_connections() == 0:
        # print('waiting')
        time.sleep(0.05)
    wb_state_pub = rospy.Publisher ("/wholebodystate", Float64MultiArray,queue_size=0) 
    while not rospy.is_shutdown():
        arm_state = np.array(rtde_r.getActualQ())
        if not car_sub_record:
            time.sleep(0.01)
            continue
        if is_new_control_msg:
            is_new_control_msg = False
            print('control msg received!!')
            pass
        car_state = np.array(car_sub_record[:3])
        wb_state = Float64MultiArray()
        wb_state.data = np.concatenate((car_state,arm_state),axis=0)
        wb_state_pub.publish(wb_state)
        time.sleep(0.01)




    
    
