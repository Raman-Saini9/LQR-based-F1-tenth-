#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import scipy
import numpy as np
import csv
import itertools
from tf.transformations import euler_from_quaternion

current_states = []
i = 2
x_pos = 0.0
y_pos = 0.0
cur_yaw = 0.0
delta = 0.0

vel = 0.1
theta = 0
beta = 0
wh_ba = 0.3302

prev_time = 0.0
prev_yaw = 0.0
time = 0.001

def next_state(itterate):

    file = open("Silverstone_centerline.csv", mode='r')
    waypoints = csv.reader(file)
    states = next(itertools.islice(waypoints, itterate, itterate + 1))
    return np.array([float(state) for state in states])  

states = next_state(2)
heading =math.atan((states[1]-y_pos)/(states[0]-x_pos))
x_tf = np.array([[states[0]],[states[1]],[heading],[0]])

def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):
        return True
    else:
        return False



def callback1(data):
    global delta
    for i in data.transforms:
        if(i.child_frame_id=="front_left_wheel"):
            delta = (euler_from_quaternion([i.transform.rotation.x, i.transform.rotation.y, i.transform.rotation.z, i.transform.rotation.w])[2])

def callback2(data):
    global current_states, x_pos, y_pos, cur_yaw, delta, vel, theta, beta, wh_ba, prev_yaw, prev_time, time
    x_pos = data.pose.position.x
    y_pos = data.pose.position.y
    cur_yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
    current_states = np.array([[x_pos], [y_pos], [cur_yaw], [delta]])
    error()

def error():
    global current_states, x_pos, y_pos, cur_yaw, delta, vel, theta, beta, wh_ba, prev_yaw, prev_time, time, x_tf,i,heading
    error_states = x_tf-current_states
    if (next_state(i)[0]-1.5 <= current_states[0] <= next_state(i)[0]+1.5 and next_state(i)[1]-1.5 <= current_states[1] <= next_state(i)[1]+1.5):
        states = next_state(i)
        heading =math.atan((states[1]-y_pos)/(states[0]-x_pos))
        x_tf = np.array([[states[0]],[states[1]],[heading],[0]])
        i = i+1

    if(prev_time==0.0):
        prev_time = rospy.get_time()
        return
    cur_time = rospy.get_time()
    time = cur_time-prev_time
    if(withinthresh(time, 0.05)):
        return
    prev_time = cur_time

    R = np.array([[1,0],[0,0.0000001]])
    Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    A = np.array([[-1,0,-vel*time*math.sin(cur_yaw),0],[0,-1,vel*time*math.cos(cur_yaw),0],[0,0,-1,(vel/wh_ba)*((1/math.cos(delta))**2)],[0,0,0,-1]])
    B = np.array( [ [math.cos(cur_yaw)*time,0],[time*math.sin(cur_yaw),0],[(math.tan(delta)*time)/wh_ba,0],[0,time]] )

    P = scipy.linalg.solve_continuous_are(A,B,Q,R)
    S = np.dot(np.linalg.inv(R),np.transpose(B))
    K = np.dot(S,P)
    U = np.dot(K,error_states)
    U[0][0] *= 100
    vel = U[0][0]
    control(U)

def control(input_array):
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    command1 = AckermannDriveStamped()
    command = AckermannDrive()
    command.speed = input_array[0][0]
    command.steering_angle = input_array[1][0]
    command1.drive = command
    pub.publish(command1)


if __name__ == '__main__':
    print("LQR started")
    rospy.init_node('myLQR', anonymous = False)
    rospy.Subscriber("/tf", TFMessage, callback1)
    rospy.Subscriber("/gt_pose", PoseStamped, callback2)
    rospy.spin()