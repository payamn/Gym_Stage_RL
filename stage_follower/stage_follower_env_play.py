#Gym stuff
import gym
from gym import error, spaces, utils
from gym.utils import seeding

#ROS stuff
import rospy
import rospkg
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty as EmptySrv
from follower_rl.msg import stage_message
from follower_rl.srv import reset_position
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
import cv2
import yaml

#The rest of stuff
import numpy as np
from math import pi,sqrt
import random
from collections import deque

MAX_ITERATIONS = 20000

REWARD_QUEUE_LEN = 100
MINIMUM_EXPLORATION_OVER_QUEUE = 200


COLLISION_REWARD = -1

CRUISE_SPEED = 0.4
AVOID_SPEED = 0.05
AVOID_TURN = 0.5
CLASSES_SPEED = [
    [CRUISE_SPEED, 0.0],  # class 0
    [AVOID_SPEED, +AVOID_TURN],  # class 1
    [AVOID_SPEED, -AVOID_TURN],  # class 2
    [0.0, +AVOID_TURN],  # class 3
    [0.0, -AVOID_TURN],  # class 4
]

# get an instance of RosPack with the default search paths
ROSPACK = rospkg.RosPack()

class StageEnvPlay(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, observation_shape=([181])):
    global CLASSES_SPEED, ROSPACK

    self.viewer = None
    self.action_space = spaces.Discrete(len(CLASSES_SPEED))
    self.observation_space = spaces.Box(low=0, high=255, shape=observation_shape[0])
    self.terminal = False
    self.sendTerminal = False
    self.readyForNewData = True
    self.minFrontDist = 3
    self.bridge = CvBridge()

    self.number_of_iteration = 0

    yaml_file = open(ROSPACK.get_path('follower_rl') + "/positions.yaml", "r")
    self.available_positions = yaml.load(yaml_file)


  def initROS(self):
    rospy.init_node('gym_node', disable_signals=True)
    # rospy.wait_for_service('/reset_positions')
    self.resetStage = rospy.ServiceProxy('/reset_positions', EmptySrv)
    self.resetRandomStage = rospy.ServiceProxy('/reset_random_positions', reset_position)

    # Subscribers:
    # rospy.Subscriber(robot_name+'/map', stage_message, self.stageCB, queue_size=10)
    rospy.Subscriber('/input_data', stage_message, self.stageCB, queue_size=10)

    # publishers:
    self.pub_vel_ = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    # self.pub_rew_ = rospy.Publisher('/lastreward',Float64,queue_size=10)

  def _step(self, action):
    if self.terminal == False:
      # self.actionToVel( action[0], action[1])
      self.actionToVelDisc(action)
      self.readyForNewData = True

    # Add whatever info you want
    wait_from = rospy.get_rostime()
    # while (self.readyForNewData == True):
    #   sleep(0.01)
    #   pass

    ## TODO: Reward
    reward = 0
    
    # rospy.loginfo('Before Reward: %f', reward)

    # make the robot moving (so that it's not stuck in the corners)

    # TODO: info
    info = {"param":np.ones(181)}
    if self.terminal == True:
      reward = COLLISION_REWARD
      #rewd = Float64()
      #rewd.data = self.ep_reward
      #self.pub_rew_.publish( rewd)
      self.sendTerminal = True
      self.number_of_iteration = 0

    if self.number_of_iteration >= MAX_ITERATIONS and self.resetRandomStartingPoint():
      self.number_of_iteration = 0
      rospy.loginfo("End of episode")
    self.number_of_iteration = self.number_of_iteration + 1

    rospy.loginfo('Final Reward: %f', reward)

    return np.ones(181), reward, random.sample([True,False],1), info

  def _reset(self):
    # self.resetRandomStartingPoint()
    self.terminal = False
    self.sendTerminal = False
    self.readyForNewData = False
    self.actionToVelDisc(0)
    # todo laser scanner fix
    return np.ones(181)

  def _render(self, mode='human', close=False):
    pass


  def _close( self):
    rospy.signal_shutdown("Done")

  def actionToVelDisc(self, action):
    if action < 0 or action >= self.action_space.n:
      rospy.logerr( "Invalid action %d", action)
    else:
      msg = Twist()
      msg.angular.x = 0
      msg.angular.y = 0
      msg.angular.z = CLASSES_SPEED[action][1]
      msg.linear.x = CLASSES_SPEED[action][0]
      msg.linear.y = 0
      msg.linear.z = 0
      # self.pub_vel_.publish(msg)
 
  def stageCB(self, data):
    try:
        cv_map = self.bridge.imgmsg_to_cv2(data.map_image, "rgb8")
        # cv_map = cv2.resize(cv_map, (299, 299,3), interpolation = cv2.INTER_AREA)#cv2.INTER_CUBIC)#

        # publish the resized map
        map_msg = self.bridge.cv2_to_imgmsg(cv_map, "rgb8")
        try:
          self.image_pub.publish(map_msg)
        except rospy.ROSException:
          pass


        # laser_scanner = np.asarray(data.laser.ranges, dtype=np.float32)
        # laser_scanner = laser_scanner / float(data.laser.range_max)
        # laser_scanner = laser_scanner.reshape(360)
        # self.laser_scanner = laser_scanner



        # self.map = cv_map

    except CvBridgeError as e:
        print(e)
    
    # print ("in callback")
    # data = np.concatenate(self.map, self.laser_scanner)
    # print (data.shape)

    if data.collision == True:
      self.terminal = 1 
    self.minFrontDist = data.minFrontDist
    self.readyForNewData = False



  def resetRandomStartingPoint(self):
    choice = random.choice(self.available_positions['positions'])
    pose = Pose()
    pose.position.x = choice['x']
    pose.position.y = choice['y']
    pose.position.z = choice['z']
    pose.orientation.w = random.uniform(-pi, pi)

    try:
      # self.resetRandomStage(pose)
      # self.clearInternals()
      return True
    except rospy.service.ServiceException:
      rospy.logerr("Unable to reset ROS")
      return False

