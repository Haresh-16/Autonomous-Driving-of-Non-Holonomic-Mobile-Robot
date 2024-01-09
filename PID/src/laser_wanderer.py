#!/usr/bin/env python

import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/car/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation' # The topic to publish controls to
POSE_TOPIC = '/car/car_pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset):
    # Store the params for later
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(CMD_TOPIC,AckermannDriveStamped,queue_size=1)# Create a publisher for sending controls
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC,LaserScan,self.wander_cb)# Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = rospy.Publisher(VIZ_TOPIC,PoseArray,queue_size=1)# Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = rospy.Subscriber(POSE_TOPIC,PoseStamped,self.viz_sub_cb)# Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):
    # Create the PoseArray to publish. Will contain N poses, where the n-th pose
    # represents the last pose in the n-th trajectory
    
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    h = cur_pose[0]
    k = cur_pose[1]
    phi = cur_pose[2]

    
    
    
    
    
    
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    
    rs = self.rollouts.shape[0]

    # Transform the last pose of each trajectory to be w.r.t the world and insert into
    # the pose array
    # YOUR CODE HERE
    for i in range(rs):
      # print("traj:", traj.shape)
      x_ = self.rollouts[i][-1][0] 
      y_ = self.rollouts[i][-1][1] 
      th = self.rollouts[i][-1][2]
      x_w = x_*np.cos(phi)-y_*np.sin(phi) + h
      y_w = x_*np.sin(phi)+y_*np.cos(phi) + k
      z_w = 0
      
      angrt = th+phi

    
      

      q_w = utils.angle_to_quaternion(angrt)
      
      posmsg = Pose()
      posmsg.position.x = x_w
      posmsg.position.y = y_w
      posmsg.position.z = z_w
      posmsg.orientation = q_w

      pa.poses.append(posmsg)
    # print("Rollout published")
    self.viz_pub.publish(pa)
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, rollout_pose, laser_msg):
    
    # Initialize the cost to be the magnitude of delta
    cost = math.fabs(delta)
    t_x = rollout_pose[0]
    t_y = rollout_pose[1]
    

    min_ang = laser_msg[0]
    max_ang = laser_msg[1]
    a_in = laser_msg[2]
    laser_ranges = laser_msg[3]
    # print("laser ranges shape:" ,len(laser_ranges))

    las_ang_ar = np.arange(min_ang, max_ang, a_in)
    # print("laser angle ranges shape:" ,len(las_ang_ar))

    prl_ang = math.atan2(t_y,t_x)
    prl_ang_dif = np.abs(prl_ang-las_ang_ar)
    prl_ind = np.argmin(prl_ang_dif)
    
    las_rng = laser_ranges[prl_ind]
    rtl = math.sqrt(t_x**2+t_y**2) #Distance between rollout pose and robot
    
    # print("Laser range :", las_rng)
    # print("Distance :", rtl)
    if math.isnan(las_rng):
      # print("Obstacle is Far away")
      cost += MAX_PENALTY
    elif las_rng - np.abs(self.laser_offset)< rtl :
      if las_rng == 0:
        # print("No obstacle ig")
        cost += MAX_PENALTY
      else :
        cost += MAX_PENALTY
    # print("Cost :", cost)
    return cost  




    # Consider the line that goes from the robot to the rollout pose
    # Compute the angle of this line with respect to the robot's x axis
    # Find the laser ray that corresponds to this angle
    # Add MAX_PENALTY to the cost if the distance from the robot to the rollout_pose 
    # is greater than the laser ray measurement - np.abs(self.laser_offset)
    # Return the resulting cost
    # Things to think about:
    #   What if the angle of the pose is less (or greater) than the angle of the
    #   minimum (or maximum) laser scan angle
    #   What if the corresponding laser measurement is NAN or 0?
    # NOTE THAT NO COORDINATE TRANSFORMS ARE NECESSARY INSIDE OF THIS FUNCTION
    
    # YOUR CODE HERE
    # pass
    
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):
    # print("Wander cb is called")

    min_ang = msg.angle_min
    max_ang = msg.angle_max
    a_in = msg.angle_increment
    laser_ranges = msg.ranges

    laser_msg = [min_ang,max_ang,a_in,laser_ranges]


    
    start = rospy.Time.now().to_sec() # Get the time at which this function started
    
    # A N dimensional matrix that should be populated with the costs of each
    # trajectory up to time t <= T
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float32) 
    traj_depth = 0
    
    # Evaluate the cost of each trajectory. Each iteration of the loop should calculate
    # the cost of each trajectory at time t = traj_depth and add those costs to delta_costs
    # as appropriate
    

    # Pseudo code
    
    while (rospy.Time.now().to_sec()-start < self.compute_time and traj_depth < self.rollouts.shape[1]):
      for ind, traj in enumerate(self.rollouts):
          delta_costs[ind] += self.compute_cost(self.deltas[ind], traj[traj_depth], laser_msg)
      traj_depth += 1 
    # YOUR CODE HERE
    
    

    # Find the delta that has the smallest cost and execute it by publishing
    # YOUR CODE HERE
    min_t_ind = np.argmin(delta_costs)
    best_delta = self.deltas[min_t_ind]


    control_msg = AckermannDriveStamped()
    control_msg.header.frame_id = '/map'
    control_msg.header.stamp = rospy.Time.now()
    control_msg.drive.speed = self.speed
    control_msg.drive.steering_angle = best_delta
    

    self.cmd_pub.publish(control_msg)


    
'''
Apply the kinematic model to the passed pose and control
You should refer to the Ackermann Drive kinematic model taught in lecture.
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  # Apply the kinematic model
  # Make sure your resulting theta is between 0 and 2*pi
  # Consider the case where delta == 0.0
  sp = control[0]
  st = control[1]
  dt = control[2]
  x_0 = pose[0]
  y_0 = pose[1]
  theta_0 = pose[2]
  x_1 = x_0 + sp*math.cos(theta_0)*dt
  y_1 = y_0 + sp*math.sin(theta_0)*dt
  theta_1 = theta_0 + (sp*math.tan(st)*dt)/car_length
  
  new_pose = np.array([x_1, y_1, theta_1])
  
  return new_pose
  # YOUR CODE HERE
  
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
You should use the kinematic_model_step function here.
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):
  # YOUR CODE HERE
  pose = init_pose
  loop_size = controls.shape[0]  #loop_size = T
  rollouts = []

  for i in range(loop_size): 
    rollout = kinematic_model_step(pose, controls[i], car_length)
    pose = rollout.copy()
    rollouts.append(pose)

  rollouts = np.array(rollouts)
  print("Rollout shape : ",rollouts.shape)  


  return rollouts

  # pass
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  print("N :",N)
  
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in range(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    print("Rollout no. :", i)
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)

  print("Rollouts shape:",rollouts.shape)  
  return rollouts, deltas

    

def main():

  rospy.init_node('laser_wanderer', anonymous=True)

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LaserWanderer class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system  
  # YOUR CODE HERE
  # speed = # Default val: 1.0
  # min_delta = # Default val: -0.341
  # max_delta = # Default val: 0.341
  # delta_incr = # Starting val: 0.34/3 (consider changing the denominator) 
  # dt = # Default val: 0.01
  # T = # Starting val: 300
  # compute_time = # Default val: 0.09
  # laser_offset = # Starting val: 1.0


  speed = rospy.get_param("~speed", 1.0)
  min_delta = rospy.get_param("~min_delta", -1.57)
  max_delta = rospy.get_param("~max_delta", 1.57)
  delta_incr = rospy.get_param("~delta_incr", 0.1133)
  print("ic type", type(delta_incr))
  dt = rospy.get_param("~dt", 0.01)
  T = rospy.get_param("~T", 300)
  compute_time = rospy.get_param("~compute_time", 0.09)
  laser_offset = rospy.get_param("~laser_offset", 3.0)

  
  # DO NOT ADD THIS TO YOUR LAUNCH FILE, car_length is already provided by teleop.launch
  car_length = rospy.get_param("/car/vesc/chassis_length", 0.33) 
  
  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()
