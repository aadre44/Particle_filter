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


