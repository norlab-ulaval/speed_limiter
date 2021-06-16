#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32 
from scipy.spatial.transform import Rotation

last_imu = Imu()

#speed limiter V2 with downhill limiter
def speed_limiter(new_cmd_vel):
    global last_imu
    global pub_cmd_vel
    global pub_pitch
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

    #dynamic state verification of the robot :

        #Check if velocity is positive or negative.
        #If initial pitch limit is not reach the velocity command is republish
        #If the max limit is reach the x velocity will be set to zero
        #Than if the pitch limit is reach the robot will be slow down in linear x velocity
    
    #FWD direction.
    if(speed > 0):
        if(pitch > pitch_up_ini):                           #acension
            pitch_delta = pitch_up_limit - pitch
            if(pitch_delta < 0):                            #(limit trepassed)
                new_cmd_vel.linear.x = 0
            else:                                           #slowdown robot (in limited range)
                new_cmd_vel.linear.x = min(max_speed_fwd*((pitch_delta/pitch_up_limit)**(0.5)), speed)

        elif(pitch < pitch_down_ini):                       #downhill
            pitch_delta = pitch_down_limit - pitch
            if(pitch_delta > 0):                            #(limit trepassed)
                new_cmd_vel.linear.x = 0
            else:                                           #slowdown robot (in limited range)
                new_cmd_vel.linear.x = min(max_speed_fwd*((pitch_delta/pitch_down_limit)**(0.5)), speed)

    #RVS direction.
    elif(speed < 0):
        if(pitch < pitch_down_ini):                         #acension
            pitch_delta = pitch_down_limit - pitch
            if(pitch_delta > 0):                            #(limit trepassed)
                new_cmd_vel.linear.x = 0
            else:                                           #slowdown robot (in limited range)
                new_cmd_vel.linear.x = max(max_speed_rvs*((pitch_delta/pitch_down_limit)**(0.5)), speed)

        elif(pitch > pitch_up_ini):                         #downhill
            pitch_delta = pitch_up_limit - pitch
            if(pitch_delta < 0):                            #(limit trepassed)
                new_cmd_vel.linear.x = 0
            else:                                           #slowdown robot (in limited range)
                new_cmd_vel.linear.x = max(max_speed_rvs*((pitch_delta/pitch_up_limit)**(0.5)), speed)

    #publish new dynamic command
    pub_cmd_vel.publish(new_cmd_vel)
    pub_pitch.publish(pitch)



def callback_imu(imu_sub):
    global last_imu
    last_imu = imu_sub


def callback_speedlimiter(cmd_vel_sub):
    global last_imu
    if(not last_imu.header.stamp.is_zero()):
        speed_limiter(cmd_vel_sub)


if __name__ == '__main__':

    global pub_cmd_vel
    global pub_pitch
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
    pub_cmd_vel = rospy.Publisher('cmd_vel_out', Twist, queue_size=3)
    pub_pitch = rospy.Publisher('speed_limiter/pitch', Float32, queue_size=1)		
    #set subscriber
    imu_sub = rospy.Subscriber("imu_topic", Imu, callback_imu,queue_size=1)
    cmd_vel_sub = rospy.Subscriber("cmd_vel_in", Twist, callback_speedlimiter, queue_size=1)
    
    rospy.spin()
