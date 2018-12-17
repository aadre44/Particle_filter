# !/usr/bin/env python
import rospy
from std_msgs.msg import String
from math import *
import random
import matplotlib.pyplot as plt
import sys
from turtlebot_ctrl.msg import TurtleBotScan
from sensor_msgs.msg import LaserScan
from turtlebot_ctrl.srv import TurtleBotControl
from std_msgs.msg import Float32, Bool
#chmod +x scripts/add_two_ints_client.py
#turtlebot_ctrl/TurtleBotScan.msg
#/turtlebot_scan
world = [[4,4], [4,-7], [-7,-7], [-7,4]]
maxX = 4
lowX = -7
maxY = 4
lowY = -7

scanN = 0.0
translationN = 0.0
turnN = 0.0

#--------------------
class RobotClass:

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
        Set the noise parameters
        '''
        self.translation_noise = float(translation)
        self.turn_noise = float(turn)
        self.scan_noise = float(scan)

#---------------------------------
# TODO: Create a subcriber that subscribes to the turtlebot_scan topic and return the list of scan data
    def sense(self, set = True):
        if set:
            self.z = listener()
            return self.z
        else:
            return listener()

    @staticmethod
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "Range Data %s", data.data)
        return data

    @staticmethod
    def listener():
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)
        data = rospy.Subscriber("turtlebot_scan", TurtleBotScan, callback)
        return data
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
                self.z = response

            # turn, and add randomness to the turning command
            self.orientation = self.orientation + float(heading) + random.gauss(0.0, self.turn_noise)
            self.orientation %= 2 * pi

            # move, and add randomness to the motion command
            dist = float(distance) + random.gauss(0.0, self.translation_noise)
            self.x = self.x + (cos(orientation) * dist)
            self.y = self.y + (sin(orientation) * dist)

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
        if math.isnan(mu):
            mu = float(50000)
        if math.isnan(x):
            x = float(50000)
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))



#-----------------------------------------

    def measurement_prob(self, bot_z):
        '''
        Calculate the measurement probability: how likely a particle is to be at the same location as the robot
        by looking at the gaussian of the robots scan data and the particles scan data
        '''
        prob = 1.0

        #compares the scan distances from the particles to that of the robot( bot_z)
        for i in range(len(bot_z)):
            particle_distance = self.z[i]
            bot_distance = bot_z[i]
            if math.isnan(particle_distance):
                particle_distance = float(50000)
            if math.isnan(bot_distance):
                bot_distance = float(50000)
            prob *= self.gaussian(particle_distance, self.scan_noise, bot_distance)
        return prob

#-------------------------------------------
class Data:
    def __init__(self):
        self.heading_data = 0.0
        self.distance = 0.0
        self.ground_truth = GroundTruth()
        self.noisy_heading = 0.0
        self.noisy_distance = 0.0
        self.scan_data = []

    def to_string(self):
        print("heading data: " + str(self.heading_data))
        print("distance: " + str(self.distance))
        print("model_name: " + self.ground_truth.model_name)
        print("pose_position_x: " + str(self.ground_truth.pose_position_x))
        print("pose_position_y: " + str(self.ground_truth.pose_position_y))
        print("pose_position_z: " + str(self.ground_truth.pose_position_z))
        print("pose_orientation_x: " + str(self.ground_truth.pose_orientation_x))
        print("pose_orientation_y: " + str(self.ground_truth.pose_orientation_y))
        print("pose_orientation_z: " + str(self.ground_truth.pose_orientation_z))
        print("pose_orientation_w: " + str(self.ground_truth.pose_orientation_w))
        print("twist_linear_x: " + str(self.ground_truth.twist_linear_x))
        print("twist_linear_y: " + str(self.ground_truth.twist_linear_y))
        print("twist_linear_z: " + str(self.ground_truth.twist_linear_z))
        print("twist_angular_x: " + str(self.ground_truth.twist_angular_x))
        print("twist_angular_y: " + str(self.ground_truth.twist_angular_y))
        print("twist_angular_z: " + str(self.ground_truth.twist_angular_z))
        print("reference_frame: " + self.ground_truth.reference_frame)
        print("noisy_heading: " + str(self.noisy_heading))
        print("noisy_distance: " + str(self.noisy_distance))
        print("scan_data: " + str(self.scan_data))


class GroundTruth:
    def __init__(self):
        self.model_name = ""
        self.pose_position_x = 0.0
        self.pose_position_y = 0.0
        self.pose_position_z = 0.0
        self.pose_orientation_x = 0.0
        self.pose_orientation_y = 0.0
        self.pose_orientation_z = 0.0
        self.pose_orientation_w = 0.0
        self.twist_linear_x = 0.0
        self.twist_linear_y = 0.0
        self.twist_linear_z = 0.0
        self.twist_angular_x = 0.0
        self.twist_angular_y = 0.0
        self.twist_angular_z = 0.0
        self.reference_frame = ""


lines = [line.rstrip('\n') for line in open("./Trajectory/trajectories_1.txt")]

list_of_trajectory = []

# data = Data()
# data.scan_data = lines[33][11:]
# data.scan_data = data.scan_data.split(", ")
# data.scan_data[-1] = data.scan_data[-1][:-1]
# print(data.scan_data)

for i in range(4, len(lines), 30):
    print(i)
    data = Data()
    data.heading_data = float(lines[i][15:])
    data.distance = float(lines[i + 1][16:])
    data.ground_truth.model_name = lines[i + 3][14:]
    data.ground_truth.pose_position_x = float(lines[i + 6][8:])
    data.ground_truth.pose_position_y = float(lines[i + 7][8:])
    data.ground_truth.pose_position_z = float(lines[i + 8][8:])
    data.ground_truth.pose_orientation_x = float(lines[i + 10][8:])
    data.ground_truth.pose_orientation_y = float(lines[i + 11][8:])
    data.ground_truth.pose_orientation_z = float(lines[i + 12][8:])
    data.ground_truth.pose_orientation_w = float(lines[i + 13][8:])
    data.ground_truth.twist_linear_x = float(lines[i + 16][8:])
    data.ground_truth.twist_linear_y = float(lines[i + 17][8:])
    data.ground_truth.twist_linear_z = float(lines[i + 18][8:])
    data.ground_truth.twist_angular_x = float(lines[i + 20][8:])
    data.ground_truth.twist_angular_y = float(lines[i + 21][8:])
    data.ground_truth.twist_angular_z = float(lines[i + 22][8:])
    data.ground_truth.reference_frame = lines[i + 23][19:]
    data.noisy_heading = float(lines[i + 25][8:])
    data.noisy_distance = float(lines[i + 27][8:])
    data.scan_data = lines[i + 29][11:]
    data.scan_data = data.scan_data.split(", ")
    data.scan_data[-1] = data.scan_data[-1][:-1]

    for j in range(len(data.scan_data)):
        data.scan_data[j] = float(data.scan_data[j])

    data.to_string()
    list_of_trajectory.append(data)
    print()
#-------------------------------------------

def vizualize(particles, robot):
    data_x = []
    data_y = []
    maxX = None
    lowX = None
    maxY = None
    lowY = None
    for i in particles:
        if i.x > maxX or maxX == None:
            maxX = i.x
        if i.x < lowX or lowX == None:
            lowX = i.x
        if i.y > maxY or maxY == None:
            maxY = i.y
        if i.y < lowY or lowY == None:
            lowY = i.y
        data_x.append(i.x)
        data_y.append(i.y)

    plt.plot(data_x, data_y, 'bo')
    plt.plot([robot.x], [robot.y], 'ro-')
    plt.axis()  # creates the x and y bounds for the graph
    plt.show()
    # _raise_not_defined()
    return

#-----------------------------------------Creating robot
# create a robot for the particle filter demo
myrobot = RobotClass()
myrobot.set(-6, 1, list_of_trajectory[0].heading_data)
myrobot.set_noise(translationN, scanN, turnN)

#----------------------------------------- Creating and Sampling set of particles

# create a set of particles
n = 2000  # number of particles
p = []  # list of particles

for i in range(n):
    x = RobotClass()
    # set random position for the particle
    x_pos = random.random( )*maxX
    y_pos = random.random()*maxY
    o = random.random(-math.pi,  math.pi)
    x.set(x_pos, y_pos, o)
    #set noise for the particle
    x.set_noise(translationN, scanN, turnN)
    p.append(x)

#-------------------------------------- Start the loop of sampling and resampling

steps = len(list_of_trajectory)  # particle filter steps
for t in range(steps):

    # move the robot and sense the environment after that
    myrobot = myrobot.move_bot(list_of_trajectory[t].heading_data, list_of_trajectory[t].distance, True)
    scanData = list_of_trajectory[t].scan_data

    p2 = []
    for i in range(n): #move all of the particles
        p2.append(p[i].move_bot(list_of_trajectory[t].heading_data, list_of_trajectory[t].distance, True))

    p = p2 #set the main set of particles to the new sampled sets

#----------------------------- Setting the weight

    # generate particle weights depending on robot's scan data
    weights = []
    for i in range(n):
        weights.append(p[i].measurement_prob(scanData))

#-------------------------------- resampling

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

    vizualize(p, myrobot)

#-------------------------------------------

