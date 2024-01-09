#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import utils as Utils

PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed
    
    self.errors = []
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)# Create a publisher to PUB_TOPIC
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, queue_size=10)# Create a subscriber to pose_topic, with callback 'self.pose_cb'
    print("LineFollower initilized")

  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    

    cur_x, cur_y, cur_theta = cur_pose
    c, s = np.cos(-cur_theta), np.sin(-cur_theta)
    rot = np.matrix([[c, -s], [s, c]])
    while len(self.plan) > 0:

      plan_x, plan_y, plan_theta = self.plan[0]
      config = np.dot(rot, np.array([[plan_x - cur_x], [plan_y - cur_y]]))

      if (config[0] < 0):
        self.plan = np.delete(self.plan, 0, 0)
      else:
        break
      

    if len(self.plan) == 0:
      return (False, 0.0)  
    

    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)

    goal_x, goal_y, goal_theta = self.plan[goal_idx]
    goal_config = np.dot(rot, np.array([[goal_x - cur_x], [goal_y - cur_y]]))
    translation_error = goal_config[1]


    rotation_error = goal_theta - cur_theta

    error = self.translation_weight * translation_error + self.rotation_weight * rotation_error


    self.errors.append(float(error[0]))
  
    return True, error
  
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() 
    

    if self.error_buff:
      last_error, last_time = self.error_buff[-1]
      deriv_error = (error - last_error) / (now - last_time)
    else:
      deriv_error = 0
    

    self.error_buff.append((error, now))
    

    integral_error = 0
    for i in range(1, len(self.error_buff)):
      e0, t0 = self.error_buff[i-1]
      e1, t1 = self.error_buff[i]
      dt = t1- t0
      e_sum = e0 + e1
      integral_error += e_sum * dt / 2

    print("Steering angle error initilized")
    return self.kp*error + self.ki*integral_error + self.kd * deriv_error
  

  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''  
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         Utils.quaternion_to_angle(msg.pose.orientation)])
    success, error = self.compute_error(cur_pose)
    
    if not success:

      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

def main():
  print("Mainfunction started")
  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  

  plan_topic = rospy.get_param("~plan_topic")
  pose_topic = rospy.get_param("~pose_topic")
  plan_lookahead = rospy.get_param('~plan_lookahead')
  translation_weight = rospy.get_param('~translation_weight')
  rotation_weight = rospy.get_param('~rotation_weight')
  kp = rospy.get_param('~kp')
  ki = rospy.get_param('~ki')
  kd = rospy.get_param('~kd')
  error_buff_length = rospy.get_param('~error_buff_length')
  speed = rospy.get_param('~speed')

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press
  

  print("plan started")
  plan_msg = rospy.wait_for_message(plan_topic, PoseArray)
  plan = [np.array([pose.position.x, pose.position.y, Utils.quaternion_to_angle(pose.orientation)]) for pose in plan_msg.poses]
  
  lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)

  rospy.spin() # Prevents node from shutting down
  print(lf.errors,"******")


if __name__ == '__main__':
  main()

