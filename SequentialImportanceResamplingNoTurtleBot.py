# !/usr/bin/env python
# import rospy
# from std_msgs.msg import String
from math import *
import random
import matplotlib.pyplot as plt
import math
import sys
# from turtlebot_ctrl.msg import TurtleBotScan
# from sensor_msgs.msg import LaserScan
# from turtlebot_ctrl.srv import TurtleBotControl
# from std_msgs.msg import Float32, Bool
#chmod +x scripts/add_two_ints_client.py
#turtlebot_ctrl/TurtleBotScan.msg
#/turtlebot_scan

world = [[4,4], [4,-7], [-7,-7], [-7,4]]
# maxX = 4, lowX = -7, maxY = 4, lowY = -7
maxX = 11
lowX = 0
maxY = 11
lowY = 0
world_size = 110
scanN = 0.1
translationN = 0.1
turnN = 0.1

landmarks = [[30.0, 70.0], [30.0, 80.0], [40.0, 70.0],
             [60.0, 80.0], [70.0, 80.0], [80.0, 80.0], [70.0, 70.0], [80.0, 70.0], [80.0, 60.0],
             [30.0, 40.0], [40.0, 40.0], [50.0, 40.0], [30.0, 30.0], [40.0, 30.0], [50.0, 30.0], [60.0, 30.0],
             [70.0, 30.0]]

class RobotClass:
    def __init__(self):
        self.x = random.random() * world_size  # robot's x coordinate
        self.y = random.random() * world_size  # robot's y coordinate
        self.orientation = random.random() * 2.0 * pi  # robot's orientation

        self.forward_noise = 0.0  # noise of the forward movement
        self.turn_noise = 0.0  # noise of the turn
        self.sense_noise = 0.0  # noise of the sensing

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')

        if -pi <= new_orientation < 0:
            new_orientation += 2 * pi

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = float(new_forward_noise)
        self.turn_noise = float(new_turn_noise)
        self.sense_noise = float(new_sense_noise)

    def sense(self):
        z = []

        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            z.append(dist)

        return z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cannot move backwards')

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)

        # cyclic truncate
        x %= world_size
        y %= world_size

        # set particle
        res = RobotClass()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)

        return res

    @staticmethod
    def gaussian(mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        prob = 1.0

        # compares the scan distances from the particles to that of the robot(bot_z)
        # TODO: Change this to take data from gazebo
        #
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.gaussian(dist, self.sense_noise, measurement[i])
        return prob


def evaluation(r, p):
    sum = 0.0

    for i in range(len(p)):
        # the second part is because of world's cyclicity
        dx = (p[i].x - r.x + (world_size / 2.0)) % world_size - (world_size / 2.0)
        dy = (p[i].y - r.y + (world_size / 2.0)) % world_size - (world_size / 2.0)
        err = sqrt(dx ** 2 + dy ** 2)
        sum += err

    return sum / float(len(p))


def visualization(robot, step, p, pr, weights):
    plt.figure("Robot in the world ", figsize = (15., 15.))
    plt.title('Particle filter, step ' + str(step))

    # draw coordinate grid for plotting
    grid = [0, world_size, 0, world_size]
    plt.axis(grid)
    plt.grid(b=True, which='major', color='0.75', linestyle='--')
    plt.xticks([i for i in range(0, int(world_size), 5)])
    plt.yticks([i for i in range(0, int(world_size), 5)])

    # draw particles
    for ind in range(len(p)):
        # particle
        circle = plt.Circle((p[ind].x, p[ind].y), 1., facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(p[ind].x, p[ind].y, 2 * cos(p[ind].orientation), 2 * sin(p[ind].orientation), alpha=1.,
                          facecolor='#994c00', edgecolor='#994c00')
        plt.gca().add_patch(arrow)

    # draw resampled particles
    for ind in range(len(pr)):
        # particle
        circle = plt.Circle((pr[ind].x, pr[ind].y), 1., facecolor='#66ff66', edgecolor='#009900', alpha=0.5)
        plt.gca().add_patch(circle)

        # particle's orientation
        arrow = plt.Arrow(pr[ind].x, pr[ind].y, 2 * cos(pr[ind].orientation), 2 * sin(pr[ind].orientation), alpha=1.,
                          facecolor='#006600', edgecolor='#006600')
        plt.gca().add_patch(arrow)

    # fixed landmarks of known locations
    for lm in landmarks:
        circle = plt.Circle((lm[0], lm[1]), 1., facecolor='#cc0000', edgecolor='#330000')
        plt.gca().add_patch(circle)

    # robot's location
    circle = plt.Circle((robot.x, robot.y), 1., facecolor='#6666ff', edgecolor='#0000cc')
    plt.gca().add_patch(circle)

    # robot's orientation
    arrow = plt.Arrow(robot.x, robot.y, 2 * cos(robot.orientation), 2 * sin(robot.orientation), alpha=0.5,
                      facecolor='#000000', edgecolor='#000000')
    plt.gca().add_patch(arrow)

    plt.savefig("figure_" + str(step) + ".png")
    plt.close()

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

    list_of_trajectory.append(data)


#-----------------------------------------Creating robot
# create a robot for the particle filter demo
myrobot = RobotClass()
myrobot.set(10, 80, list_of_trajectory[0].heading_data)
myrobot.set_noise(translationN, scanN, turnN)

#----------------------------------------- Creating and Sampling set of particles
# create a set of particles
n = 1000  # number of particles
p = []  # list of particles

for i in range(n):
    x = RobotClass()
    #set noise for the particle
    x.set_noise(translationN, scanN, turnN)
    p.append(x)

#-------------------------------------- Start the loop of sampling and resampling

steps = len(list_of_trajectory)  # particle filter steps
for t in range(0, 5):

    # move the robot and sense the environment after that
    myrobot = myrobot.move(list_of_trajectory[t].heading_data, list_of_trajectory[t].distance)
    scanData = list_of_trajectory[t].scan_data

    p2 = []
    # move all of the particles
    for i in range(n):
        p2.append(p[i].move(list_of_trajectory[t].heading_data, list_of_trajectory[t].distance))

    # set the main set of particles to the new sampled sets
    p = p2
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
    maxW = max(weights) #max weight

    for i in range(n):
        beta += random.random() * 2.0 * maxW

        while beta > weights[index]:
            beta -= weights[index]
            index = (index + 1) % n
        p3.append(p[index])

#-------------------------------------------

    #  here we get a set of co-located particles
    p = p3

    visualization(myrobot, t, p2, p, weights)

#-------------------------------------------

