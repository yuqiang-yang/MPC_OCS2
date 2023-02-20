#!/usr/bin/python3
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
control_time = 0.2

#car state subsciber callback function
def car_sub_callback(odom:Odometry):
    global car_sub_record
    x = odom.pose.pose.orientation.x
    y = odom.pose.pose.orientation.y
    z = odom.pose.pose.orientation.z
    w = odom.pose.pose.orientation.w
    theta = atan2(2*(w*z+x*y),1-2*(y**2+z**2))
    car_sub_record = [odom.pose.pose.position.x, odom.pose.pose.position.y, theta ,odom.twist.twist.linear.x, odom.twist.twist.angular.z]

def wholebodycontrolcallback(msg:Float64MultiArray):
    global is_new_control_msg
    global arm_desired_vel
    global base_desired_vel
    is_new_control_msg = True
    control_time = 0.2
    base_desired_vel = msg.data[:2]
    arm_desired_vel = msg.data[2:]
    # print(base_desired_vel,arm_desired_vel)
    pass
class WBController():
    def __init__(self,base_pub,rtde_c,config:dict) -> None:
        self.base_pub = base_pub
        self.rtde_c = rtde_c
        self.dt = 0.01
        self.current_arm_vel = np.zeros(6)
        self.current_car_vel = np.zeros(2)
        if config:
            self.car_linear_acc = config['car_linear_acc']
            self.car_angular_acc = config['car_angular_acc']
            self.car_linear_vel = config['car_linear_vel']
            self.car_angular_vel = config['car_angular_vel']
            self.arm_max_acc = config['arm_max_acc']
            self.arm_max_vel = config['arm_max_vel']

        else:
            print("the limits are not set. It's unsafe!!!")
            sys.exit()

    def control(self,base_vel:np.array,joint_vel:np.array):
        current_arm_vel = self.current_arm_vel
        current_car_vel = self.current_car_vel
        arm_state = rtde_r.getActualQ()
        vel = Twist()
        vel.linear.x = base_vel[0]
        vel.angular.z = base_vel[1]
        dt = self.dt
        joint_vel = np.array(joint_vel)
        for i in range(6):
            if abs(joint_vel[i] - current_arm_vel[i]) > self.arm_max_acc*dt:    #acc
                joint_vel[i] = current_arm_vel[i] + self.arm_max_acc*dt*np.sign(joint_vel[i]-current_arm_vel[i])
            if abs(joint_vel[i]) > self.arm_max_vel:
                joint_vel[i] = self.arm_max_vel*np.sign(joint_vel[i])
        # joint_vel[5] = 0
        # print('current',current_arm_vel,'desired',joint_vel)
        if abs(current_car_vel[0]-vel.linear.x) > self.car_linear_acc*dt:
            vel.linear.x = current_car_vel[0] + np.sign(vel.linear.x-current_car_vel[0])*self.car_linear_acc*dt
        if abs(current_car_vel[1]-vel.angular.z) > self.car_angular_acc*dt:
            vel.angular.z = current_car_vel[1] + np.sign(vel.angular.z-current_car_vel[1])*self.car_angular_acc*dt

        if abs(vel.linear.x) > self.car_linear_vel:
            vel.linear.x = self.car_linear_vel*np.sign(vel.linear.x)
            # print('the linear vel is too large')
        if abs(vel.angular.z) > self.car_angular_vel:
            vel.angular.z = self.car_angular_vel*np.sign(vel.angular.z)
            # print('the angular vel is too large')
        self.current_arm_vel = joint_vel
        self.current_car_vel = np.array((vel.linear.x,vel.angular.z))
        base_pub.publish(vel)  
        # self.rtde_c.servoJ(np.array(arm_state) + joint_vel*dt,0,0,0.05,0.15,1000)
        t1 = time.time()
        self.rtde_c.speedJ(joint_vel,time=dt)

if __name__ == '__main__':
    rospy.init_node('mue_interface_node', anonymous=True)
    ur_ip = rospy.get_param('~ur_ip','192.168.100.3')
    ur_control_activate = rospy.get_param('~ur_control_activate',True)
    base_pub = rospy.Publisher ("/mobile_base/cmd_vel", Twist,queue_size=0) 
    base_sub = rospy.Subscriber("/mobile_base/odom",Odometry,car_sub_callback)
    
    wb_control_sun =  rospy.Subscriber("/wholebodycontrol",Float64MultiArray,wholebodycontrolcallback)
    velocity_and_acc_limit = rospy.get_param('velocity_and_acc_limit',True)
    if velocity_and_acc_limit:
        WBControllerConfig = dict()
        WBControllerConfig['car_linear_acc'] = rospy.get_param('car_linear_acc',0.5)
        WBControllerConfig['car_angular_acc'] = rospy.get_param('car_angular_acc',0.2)
        WBControllerConfig['car_linear_vel'] = rospy.get_param('car_linear_vel',0.5)
        WBControllerConfig['car_angular_vel'] = rospy.get_param('car_angular_vel',0.2)
        WBControllerConfig['arm_max_acc'] = rospy.get_param('arm_max_acc',0.5)
        WBControllerConfig['arm_max_vel'] = rospy.get_param('arm_max_vel',0.5)
    else:
        WBControllerConfig = dict()
    if ur_control_activate:
        for i in range(3):
            try:
                rtde_c = rtde_control.RTDEControlInterface(ur_ip)
                break
            except Exception:
                time.sleep(3)
                
                print('keep trying to connect RTDE Control')
                if i == 2:
                    sys.exit()    
    else:
        rtde_c = None
    rtde_r = rtde_receive.RTDEReceiveInterface(ur_ip)
    
    wbc = WBController(base_pub,rtde_c,WBControllerConfig)
    time.sleep(0.1)
    while base_sub.get_num_connections() == 0:
        print('waiting')
        time.sleep(0.2)
    wb_state_pub = rospy.Publisher ("/wholebodystate", Float64MultiArray,queue_size=0) 
    cnt = 0

    while not rospy.is_shutdown() :
        # print('out',control_time)
        arm_state = np.array(rtde_r.getActualQ())
        arm_velocity = np.array(rtde_r.getActualQd())
        # print(rtde_r.getActualQd())
        # if not car_sub_record:
        #     time.sleep(0.01)
        #     continue
        # if is_new_control_msg or control_time > 0:
        #     print('int')
        #     control_time -= 0.01
        #     is_new_control_msg = False
        if ur_control_activate and rtde_c.isProgramRunning():
            wbc.control(base_desired_vel,arm_desired_vel)
            # print('control msg received!!')
            
        car_state = np.array(car_sub_record[:3])
        x_dot = car_sub_record[3] * cos(car_sub_record[2])
        y_dot = car_sub_record[3] * sin(car_sub_record[2])
        theta_dot = car_sub_record[4]
        
        wb_state = Float64MultiArray()
        wb_state.data = np.concatenate((car_state,arm_state,[x_dot,y_dot,theta_dot],arm_velocity,[rospy.Time().now().to_time()]),axis=0)
        wb_state_pub.publish(wb_state)
        time.sleep(0.01)
    rtde_c.speedStop();



    
    
