#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from scipy.spatial.transform import Rotation

last_imu = Imu()


def speed_limiter(new_cmd_vel):
    global last_imu
    global pub_cmd_vel
    global pitch_up_ini
    global pitch_up_limit
    global pitch_down_ini
    global pitch_down_limit
    global max_speed_fwd
    global max_speed_rvs

    orientation_imu = Rotation.from_quat([last_imu.orientation.x, last_imu.orientation.y, last_imu.orientation.z, last_imu.orientation.w])
    _, pitch, _ = orientation_imu.as_euler('ZYX', degrees=True)

    #get speed
    speed = new_cmd_vel.linear.x 

    #dynamic state verification of the robot 
    if(speed > 0):
        if(pitch > pitch_up_ini):
            pitch_delta = pitch_up_limit - pitch
            if(pitch_delta < 0): 
                new_cmd_vel.linear.x = 0
            else:
                new_cmd_vel.linear.x = min(max_speed_fwd*(pitch_delta/pitch_up_limit), speed)

    elif(speed < 0):

        if(pitch < pitch_down_ini):
            pitch_delta = pitch_down_limit - pitch
            if(pitch_delta > 0): 
                new_cmd_vel.linear.x = 0
            else:
                new_cmd_vel.linear.x = max(max_speed_rvs*(pitch_delta/pitch_down_limit), speed)
    
    #publish new dynamic command
    pub_cmd_vel.publish(new_cmd_vel)



def callback_imu(imu_sub):
    global last_imu
    last_imu = imu_sub


def callback_speedlimiter(cmd_vel_sub):
    global last_imu
    if(not last_imu.header.stamp.is_zero()):
        speed_limiter(cmd_vel_sub)


if __name__ == '__main__':

    global pub_cmd_vel
    global pitch_up_ini
    global pitch_up_limit
    global pitch_down_ini
    global pitch_down_limit
    global max_speed_fwd
    global max_speed_rvs
    rospy.init_node('speed_limiter')
    rospy.loginfo("Speed Limiter running...")

    # get the limit parameter of the robot
    pitch_up_ini = rospy.get_param('~pitch_up_ini')         #initial positive limit
    pitch_up_limit = rospy.get_param('~pitch_up_limit')     #limit pitch tolerated
    pitch_down_ini = rospy.get_param('~pitch_down_ini')     #initial negative limit
    pitch_down_limit = rospy.get_param('~pitch_down_limit') #limit pitch tolerated
    max_speed_fwd = rospy.get_param('~max_speed_fwd')       #foward speed limit
    max_speed_rvs = rospy.get_param('~max_speed_rvs')       #reverse speed limit

    #set publisher

    pub_cmd_vel = rospy.Publisher('speed_limiter/cmd_vel_out', Twist, queue_size=3)	
    #set subscriber
    imu_sub = rospy.Subscriber("imu_topic", Imu, callback_imu)
    cmd_vel_sub = rospy.Subscriber("speed_limiter/cmd_vel_in", Twist, callback_speedlimiter)
    
    rospy.spin()
