# !/usr/bin/env python
import rospy
from std_msgs.msg import String
from math import *
import random
import matplotlib.pyplot as plt
import sys
from TurtleBotScan.msg import String
#chmod +x scripts/add_two_ints_client.py
#turtlebot_ctrl/TurtleBotScan.msg
#/turtlebot_scan
# size of one dimension (in meters)
world_size = 100.0
world = [[0,0], [1,1], [2,2], [3,3]]
maxX = 4
lowX = -7
maxY = 4
lowY = -7

scanN = 0.0
translationN = 0.0
turnN = 0.0

#--------------------
class RobotClass:
    '''
    Class for the robot model used in this demo
    '''

    def __init__(self):
        self.x = random.random() * world_size  # robot's x coordinate
        self.y = random.random() * world_size  # robot's y coordinate
        self.orientation = random.random() * 2.0 * pi  # robot's orientation

        self.z = [] #robots current scan data
        self.truePosition = []

        self.translation_noise = 0.0  # noise of the forward movement
        self.turn_noise = 0.0  # noise of the turn
        self.scan_noise = 0.0  # noise of the sensing

#---------------------

    def set(self, x, y, orientation):
        '''
        Set robot's initial position and orientation
        :param new_x: new x coordinate
        :param new_y: new y coordinate
        :param new_orientation: new orientation
        '''

        if x < lowX or x >= maxX:
            print('X coordinate out of bound')
            exit()
        if y < lowY or y >= maxY:
            print('Y coordinate out of bound')
            exit()
        if orientation < 0 or orientation >= 2 * pi:
            print('Orientation must be in [0..2pi]')
            exit()

        self.x = float(x)
        self.y = float(y)
        self.orientation = float(orientation)

#---------------------------

    def set_noise(self, translation, turn, scan):
        '''
        Set the noise parameters, changing them is often useful in particle filters
        :param new_forward_noise: new noise value for the forward movement
        :param new_turn_noise:    new noise value
         for the turn
            :param new_sense_noise:  new noise value for the sensing
        '''
        self.translation_noise = float(translation)
        self.turn_noise = float(turn)
        self.scan_noise = float(scan)

#---------------------------------

    def sense(self, set = True):
        if set:
            self.z = listener()
        else:
            return listener()

    @staticmethod
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "Range Data %s", data.data)

    @staticmethod
    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("chatter", String, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

#-----------------------------------

    def move_bot(self, heading, distance, ground_bool=True):
        try:
            move_robot = rospy.ServiceProxy('/turtlebot_control', TurtleBotControl)
            response = move_robot(Float32(heading), Float32(distance), Bool(ground_bool))
            if not response:
                print('unable to move robot')
                return None
            if ground_bool == True:
                self.truePosition = response[0]
                self.z = response[1]
            else:
                self.z = response[0]

            # turn, and add randomness to the turning command
            self.orientation = self.orientation + float(heading) + random.gauss(0.0, self.turn_noise)
            self.orientation %= 2 * pi

            # move, and add randomness to the motion command
            dist = float(distance) + random.gauss(0.0, self.translation_noise)
            self.x = self.x + (cos(orientation) * distance)
            self.y = self.y + (sin(orientation) * distance)

            # cyclic truncate
            self.x %= world_size
            self.y %= world_size

            return response

        except rospy.ServiceException:
            print("service call failed")
        return None

#------------------------------------
    @staticmethod
    def gaussian( mu, sigma, x):
        '''
        calculates the probability of x for 1 - dim Gaussian with mean mu and var.sigma
        :param mu:    distance to the landmark
        :param sigma: standard deviation
        :param x:     distance to the landmark measured by the robot
        :return gaussian value
        '''
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))



#-----------------------------------------

    def measurement_prob(self, bot_z):
        '''
        Calculate the measurement probability: how likely a particle is to be at the same location as the robot
         by looking at the gaussian of the robots scan data and the particles scan data
        :param measurement: current measurement
        :return probability
        '''
        prob = 1.0

        #compares the scan distances from the particles to that of the robot( bot_z)
        for i in range(len(bot_z)):
            particle_distance = self.z[i]
            prob *= self.gaussian(particle_distance, self.scan_noise, bot_z[i])
        return prob

#-------------------------------------------




#-----------------------------------------Creating robot
# create a robot for the particle filter demo
myrobot = RobotClass()
myrobot.set(-6, 1, math.pi)
myrobot.set_noise(translationN, scanN, turnN)

#----------------------------------------- Creating and Sampling set of particles

# create a set of particles
n = 1000  # number of particles
p = []  # list of particles

for i in range(n):
    x = RobotClass()
    # set random position for the particle
    x_pos = random.random(lowX, maxX)
    y_pos = random.random(lowY, maxY)
    o = random.random(lowY, maxY)
    x.set(x_pos, y_pos, o)
    #set noise for the particle
    x.set_noise(translationN, scanN, turnN)
    p.append(x)

#-------------------------------------- Start the loop of sampling and resampling

steps = 50  # particle filter steps
for t in range(steps):

    # move the robot and sense the environment after that
    myrobot = myrobot.move_bot(0.1, 5., True)

    p2 = []
    for i in range(n): #move all of the particles
        p2.append(p[i].move_bot(0.1, 5., True))

    p = p2 #set the main set of particles to the new sampled sets

#----------------------------- Setting the weight

    # generate particle weights depending on robot's scan data
    weights = []
    for i in range(n):
        weights.append(p[i].measurement_prob(myrobot.z))

#-------------------------------- QUESTION resampling

    # resampling with a sample probability proportional
    # to the importance weight
    p3 = []

    index = int(random.random() * n) #pick a random index
    beta = 0.0 #will be used to prioritize the higher weighted particles
    maxW = max(w) #max weight

    for i in range(n):
        beta += random.random() * 2.0 * maxW

        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % n
        p3.append(p[index])

#-------------------------------------------

    #  here we get a set of co-located particles
    p = p3

    print('Step = ', t, ', Evaluation = ', evaluation(myrobot, p))

#-------------------------------------------

