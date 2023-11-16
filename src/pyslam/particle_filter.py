import random
import math
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor, LinearRegression
import numpy as np

withOdom = True
fitplot = True

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta  # Orientation angle
        self.weight = weight
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta

def load_keypoints(filename):
    keypoints = {}
    with open(filename, 'r') as file:
        for i, line in enumerate(file):
            x, y = line.strip().split(',')
            keypoints[i] = (float(x), float(y))
            # keypoints.append(float(x),float(y))
        # print(keypoints)
    return keypoints

def rotate_point(point, angle):
    x, y = point
    new_x = x * math.cos(angle) - y * math.sin(angle)
    new_y = x * math.sin(angle) + y * math.cos(angle)
    return new_x, new_y

def integrate_odometry(previous_pose, delta, angle):
    # Implement your odometry integration here
    # This is a simple example, and you might need to adjust it based on your robot's kinematics
    return previous_pose + delta * math.cos(angle)

def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def update_particles(particles, keypoints, observed_positions, odometry):
    for particle in particles:
        # Integrate odometry information into the particle's pose
        # The specific integration will depend on your motion model
        if withOdom:
            particle.x = integrate_odometry(particle.prev_x, odometry[0], particle.prev_theta)
            particle.y = integrate_odometry(particle.prev_y, odometry[1], particle.prev_theta)
            particle.theta = integrate_odometry(particle.prev_theta, odometry[2], 0)

        # Simulate sensor measurements for each particle
        particle_positions = {}
        for keypoint, true_position in keypoints.items():
            rotated_true_position = rotate_point(true_position, particle.theta)
            particle_positions[keypoint] = (
                particle.x + rotated_true_position[0],
                particle.y + rotated_true_position[1]
            )

        # Calculate particle weights based on the differences between observed and true positions
        weights = [1.0 / calculate_distance(observed_positions[keypoint], particle_positions[keypoint]) for keypoint in keypoints]

        # Update particle weight based on the average weight from all keypoints
        particle.weight = sum(weights) / len(weights)

        # Update the previous pose for the next iteration
        particle.prev_x = particle.x
        particle.prev_y = particle.y
        particle.prev_theta = particle.theta

        # Normalize weights to make them probabilities
    total_weight = sum(particle.weight for particle in particles)
    for particle in particles:
        particle.weight /= total_weight

# def update_particles(particles, keypoints, observed_positions):
#     for particle in particles:
#         # Simulate sensor measurements for each particle
#         particle_positions = {}
#         for keypoint, true_position in keypoints.items():
#             rotated_true_position = rotate_point(true_position, particle.theta)
#             particle_positions[keypoint] = (
#                 particle.x + rotated_true_position[0],
#                 particle.y + rotated_true_position[1]
#             )

#         # Calculate particle weights based on the differences between observed and true positions
#         weights = [1.0 / calculate_distance(observed_positions[keypoint], particle_positions[keypoint]) for keypoint in keypoints]
        
#         # Update particle weight based on the average weight from all keypoints
#         particle.weight = sum(weights) / len(weights)
#         # if particle.weight > 1.0:
#         print(observed_positions[keypoint])
#         print(particle_positions[keypoint])
#     # Normalize weights to make them probabilities
#     total_weight = sum(particle.weight for particle in particles)
#     for particle in particles:
#         particle.weight /= total_weight

def resample(particles):
    new_particles = []
    max_weight_particle = max(particles, key=lambda p: p.weight)

    for _ in particles:
        # Select a particle based on its weight
        selected_particle = random.choices(particles, weights=[p.weight for p in particles])[0]

        # Create a new particle with the same pose as the selected particle
        new_particle = Particle(selected_particle.x, selected_particle.y, selected_particle.theta, 1.0)

        # Introduce some random noise to the new particle's pose
        new_particle.x += random.gauss(0, 0.1)
        new_particle.y += random.gauss(0, 0.1)
        new_particle.theta += random.gauss(0, 0.1)

        new_particles.append(new_particle)

    return new_particles

# def estimate_robot_position(particles):
#     # Extract the x, y, and theta positions of particles
#     particle_positions = np.array([(particle.x, particle.y, particle.theta) for particle in particles])

#     # Fit a RANSAC model to find the best fit line (pose)
    
#     ransac = RANSACRegressor()#LinearRegression()
#     # ransac = linear_model.RANSACRegressor()
#     ransac.set_params(
#         min_samples=30,           # Minimum number of samples to fit the model
#         residual_threshold=0.1,   # Maximum residual for a data point to be considered an inlier
#         max_trials=10           # Maximum number of RANSAC iterations
#     )
#     ransac.fit(particle_positions[:, :2], particle_positions[:, 2])
    

#     # Extract the best fit parameters (pose)
#     estimated_x, estimated_y, estimated_theta = ransac.estimator_.intercept_, *ransac.estimator_.coef_

#     return estimated_x, estimated_y, estimated_theta

def estimate_robot_position(particles):
    # Calculate the weighted average of particle poses to estimate the robot's position
    x_sum = sum(particle.x * particle.weight for particle in particles) /len(particles)
    y_sum = sum(particle.y * particle.weight for particle in particles) /len(particles)
    theta_sum = sum(particle.theta * particle.weight for particle in particles) /len(particles)

    return -x_sum, -y_sum, theta_sum



def plot_results(keypoints, observed_positions, particles, estimated_position):
    # Plot known keypoints in black
    plt.clf()
    plt.scatter(*zip(*keypoints.values()), color='black', label='Known Keypoints')

    # Estimate the robot position based on the particles
    #estimated_position = estimate_robot_position(particles)

    if fitplot:
        # Apply the estimated shift to the observed positions
        shifted_observed_positions = {
            keypoint: (
                obs_pos[0] + estimated_position[0],
                obs_pos[1] + estimated_position[1]
            )
            for keypoint, obs_pos in observed_positions.items()
        }
        # Plot shifted observed positions in red
        plt.scatter(*zip(*shifted_observed_positions.values()), color='red', marker='x', label='Shifted Observed Positions')
    else:
        plt.scatter(*zip(*observed_positions.values()), color='red', marker='x', label='Observed Positions')

    # for _ in particles:
    #     plt.scatter(particles[_][0], estimated_position[_][1], color='blue', marker='o', label='Estimated Robot Position')

    # Plot estimated robot position based on particles in blue
    plt.scatter(estimated_position[0], estimated_position[1], color='blue', marker='o', label='Estimated Robot Position')

    plt.legend()
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.title('Robot Localization with Particle Filter')
    plt.pause(0.2)


if __name__ == "__main__":
    # keypoint_file = "/home/simon/autonomousRacecar/src/pyslam/keypoints.txt"
    keypoint_file = "/home/simon/autonomousRacecar/src/pyslam/webotskeypointsmap.txt"
    keypoints = load_keypoints(keypoint_file)

    # Initialize particles randomly across the map
    num_particles = 500
    fieldsize = 5
    particles = [Particle(random.uniform(-fieldsize, fieldsize), random.uniform(-fieldsize, fieldsize), random.uniform(-math.pi, math.pi), 1.0) for _ in range(num_particles)]

    # Odometry readings
    # odometry = (-0.302, 0.792, 0.02)
    odometry = (0.0,0.0,0.0)
    
    # Observed positions in the robot's local frame
    observed_positions = {
        0: (2.8024442838334487,0.002999014485575261),
        1: (2.80327375955996,0.0723252306825612),
        2: (2.804078534501612,0.14177976951655594),
        3: (2.804858617310802,0.21144620521146645),
        4: (2.8056154364117663,0.28140919757586247),
        5: (2.806349111292029,0.35175475122685984),
        6: (2.8070613836234317,0.42257093172823135),
        7: (2.8077516374884888,0.4939478284721877),
        8: (2.808422084850675,0.5659787826725902),
        9: (2.8090722991429384,0.6387600195931217),
        10: (2.809702644832621,0.7123917261913025),
        11: (2.810314097421104,0.7869786941308751),
        12: (2.810907544926458,0.8626308290015876),
        13: (2.8114825946450965,0.9394634429856084),
        14: (2.8120599053926068,1.017605931423041),
        15: (2.8126002573368645,1.0971740203512137),
        16: (2.813124705028121,1.1783125367652452),
        17: (2.8136320676578883,1.261168102968811),
        18: (2.814124438908099,1.3458993050196453),
        19: (2.7712437374419574,1.4106058496469982),
        20: (2.6485593525079696,1.4316764889227298),
        21: (2.536296789496708,1.4531390889456683),
        22: (2.433181775666589,1.4751321913857867),
        23: (2.2499463305156238,1.441292871992339),
        24: (2.1684607941382814,1.4659415180761661),
        25: (2.0217715773637437,1.4408607513310687),
        26: (1.955756910787651,1.468051646999202),
        27: (1.8356696193304918,1.4502255953343897),
        28: (1.7294926405672897,1.4371907266656156),
        29: (1.6809786745792688,1.4686259366427912),
        30: (1.5915251353017883,1.4614046327206158),
        31: (1.5111202597316407,1.4580515648484296),
        32: (1.419617345041167,1.4392094230705121),
        33: (1.3517023182602048,1.4398828845954728),
        34: (1.2867986910961609,1.4405140343589746),
        35: (1.2246282926541843,1.4410960258200947),
        36: (1.1649671817858738,1.4416511152859643),
        37: (1.1075904354215753,1.442171752668613),
        38: (1.0522972685755247,1.4426513194903072),
        39: (0.9989262438415324,1.4431106819562716),
        40: (0.947304517499732,1.4435322480916086),
        41: (0.8973006717702476,1.4439371927571745),
        42: (0.8487702222024734,1.4443072710637632),
        43: (0.8015989506881169,1.4446536905740692),
        44: (0.7556759983816265,1.4449775539382084),
        45: (0.7109040158996971,1.4452894913101262),
        46: (0.6671791845186474,1.4455717130912569),
        47: (0.6244187031377141,1.4458338155624906),
        48: (0.5825416263492047,1.4460773430803104),
        49: (0.5414718866764606,1.4463025141377703),
        50: (0.5011418897656436,1.4465198384936577),
        51: (0.46147800820892315,1.44671080969091),
        52: (0.42242066699815994,1.446884817778617),
        53: (0.3839105575209882,1.447043168125752),
        54: (0.3458909951059096,1.4471860384384039),
        55: (0.30830792688220854,1.4473135353985007),
        56: (0.271109672063663,1.4474260537579626),
        57: (0.2342465158784264,1.4475239053610045),
        58: (0.19767045846147083,1.4476074968423074),
        59: (0.1613348739459905,1.447676772394797),
        60: (0.1251943790621213,1.447731927928763),
        61: (0.08920455529718734,1.4477732803862187),
        62: (0.053321691515739705,1.4478008727919807),
        63: (0.017502600005570825,1.447814651216041),
        64: (-0.01829558518634143,1.447814385143057),
        65: (-0.05411562972921329,1.4478005218853216),
        66: (-0.09000039687577209,1.4477726368714436),
        67: (-0.12599307351099484,1.4477308440235002),
        68: (-0.1621373937490808,1.4476752778504913),
        69: (-0.1984777563890756,1.4476053233527446),
        70: (-0.2350596342072263,1.4475213124977966),
        71: (-0.27192951346210065,1.4474222952495241),
        72: (-0.30913558342854075,1.4473088568792463),
        73: (-0.34672748982435186,1.4471801475593284),
        74: (-0.38475691937167755,1.447035703617939),
        75: (-0.42327799655666126,1.44687557649579),
        76: (-0.4623473898603774,1.4466993971750333),
        77: (-0.5020245735988468,1.4465062331384007),
        78: (-0.5423761077589357,1.4463051325327765),
        79: (-0.583462094569004,1.4460769347048525),
        80: (-0.6253566231173389,1.4458297355685414),
        81: (-0.6681358210610294,1.4455633300769921),
        82: (-0.7118808599086045,1.4452764995087943),
        83: (-0.756684535387639,1.4449785362298304),
        84: (-0.8026311028737054,1.4446485127413995),
        85: (-0.8498278263375356,1.444295647143213),
        86: (-0.8983852522902445,1.4439180684997752),
        87: (-0.9484306360827249,1.4435242574352967),
        88: (-1.0000837344870035,1.4430934980050896),
        89: (-1.053502292013361,1.442643319052923),
        90: (-1.1088319309526462,1.4421527096636577),
        91: (-1.166262737355034,1.4416386980910427),
        92: (-1.2047214521606915,1.4160980933035974),
        93: (-1.2302022112709106,1.3756397571658496),
        94: (-1.2233289229021258,1.3017055456042768),
        95: (-1.2197391320596218,1.2352189540293352),
        96: (-1.205122890357585,1.1615267379024306),
        97: (-1.20498529573192,1.1052519903147995),
        98: (-1.2048432415957435,1.0514763044949862),
        99: (-1.2046888602746455,0.9999693576294043),
        100: (-1.2045375158717162,0.9505443263201444),
        101: (-1.2043814830141149,0.9030152468671434),
        102: (-1.204212551347347,0.857215000416604),
        103: (-1.204046505477108,0.8130095152938873),
        104: (-1.2038672788129725,0.7702557223528786),
        105: (-1.2036906743401006,0.7288441005878794),
        106: (-1.203500549867673,0.6886552367703551),
        107: (-1.2033045818989638,0.6495940010090775),
        108: (-1.2031027043828397,0.6115685202528801),
        109: (-1.2029030868088353,0.5744981191681855),
        110: (-1.2026889479379927,0.5382964033324444),
        111: (-1.2024689179738024,0.5028953119312138),
        112: (-1.202241802790818,0.46822699556548214),
        113: (-1.2020083173642349,0.43422892628234105),
        114: (-1.365819549417171,0.45556044833081477),
        115: (-1.4748954626093844,0.45174254480120357),
        116: (-1.6029543882028483,0.44793247012595955),
        117: (-1.755427401771697,0.4440603991339232),
        118: (-1.9400355749897196,0.4400309723558513),
        119: (-2.1681387007786674,0.43570762179884515),
        120: (-2.2021509743644825,0.3861656312452077),
        121: (-2.201608497948113,0.3301940473327442),
        122: (-2.2010491118378233,0.27466073482247955),
        123: (-2.20047253314578,0.21949497451765965),
        124: (-2.199878532196406,0.16462784648027176),
        125: (-2.199265894030701,0.10999181281820411),
        126: (-2.1986346763653826,0.055520552957211915),
        127: (-2.1979836801338517,0.001148472043754379)
    }

    # Update and resample particles based on sensor measurements
    for i in range(50):
        update_particles(particles, keypoints, observed_positions, odometry)
        withOdom = False
        particles = resample(particles)
        print(len(particles))
        # Estimate the robot's position based on the particles
        estimated_position = estimate_robot_position(particles)
        plot_results(keypoints, observed_positions, particles, estimated_position)

    
    
    

    print("Estimated Robot Position:", estimated_position)
    
