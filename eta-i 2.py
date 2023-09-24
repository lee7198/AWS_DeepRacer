import math


class SETTINGS:
    verbose = False
    debug = False
    REWARD_FOR_FASTEST_TIME = 500  # should be adapted to track length and other rewards. finish_reward = max(1e-3, (-self.REWARD_FOR_FASTEST_TIME / (15*(self.STANDARD_TIME - self.FASTEST_TIME)))*(steps-self.STANDARD_TIME*15))
    STANDARD_TIME = 10  # seconds (time that is easily done by model)
    FASTEST_TIME = 8  # seconds (best time of 1st place on the track)
    REWARD_PER_STEP_FOR_FASTEST_TIME = 1
    STAGE = 1
    DISTANCE_MULTIPLIER = 1
    STEERING_MULTIPLIER = 0.5
    SPEED_MULTIPLIER = 0
    PROGRESS_MULTIPLIER = 0.5


class P:
    all_wheels_on_track = None
    x = None
    y = None
    distance_from_center = None
    is_left_of_center = None
    heading = None
    progress = None
    steps = None
    speed = None
    steering_angle = None
    track_width = None
    waypoints = None
    closest_waypoints = None
    is_offtrack = None


class G:
    reward = 0
    distance_reward = 0
    speed_reward = 0
    steps_reward = 0


def reward_function(params):
    if SETTINGS.STAGE == 2:
        SETTINGS.DISTANCE_MULTIPLIER = 1
        SETTINGS.STEERING_MULTIPLIER = 0.5
        SETTINGS.SPEED_MULTIPLIER = 1
        SETTINGS.PROGRESS_MULTIPLIER = 0.5

    read_params(params)

    #################### RACING LINE ######################
    # Optimal racing line for the Spain track
    # Each row: [x,y,speed,timeFromPreviousPoint]
    racing_track = [
        [0.63069109, 2.80611932, 1.34807, 0.13673],
        [0.63367125, 2.69079621, 1.354845, 0.13571],
        [0.6467188, 2.57569291, 1.36162, 0.13469],
        [0.66972231, 2.46183988, 1.369235, 0.134565],
        [0.70251506, 2.35022569, 1.37685, 0.13444],
        [0.74487589, 2.24177514, 1.386975, 0.13498],
        [0.79652923, 2.1373277, 1.3971, 0.13552],
        [0.85714459, 2.03761659, 1.40827, 0.136825],
        [0.92633571, 1.94324872, 1.41944, 0.13813],
        [1.00365975, 1.85468625, 1.42929, 0.14039],
        [1.08861721, 1.77223051, 1.43914, 0.14265],
        [1.1806537, 1.69600972, 1.448485, 0.14571],
        [1.27916562, 1.6259728, 1.45783, 0.14877],
        [1.38351227, 1.56189156, 1.46537, 0.1525],
        [1.4930358, 1.50337335, 1.47291, 0.15623],
        [1.60708637, 1.44988322, 1.479585, 0.159915],
        [1.72504192, 1.40077175, 1.48626, 0.1636],
        [1.84630443, 1.35530161, 1.78861, 0.14245],
        [1.97025603, 1.31266612, 2.09096, 0.1213],
        [2.09617545, 1.2719922, 2.288305, 0.11264],
        [2.2231517, 1.23232374, 2.48565, 0.10398],
        [2.35576976, 1.190681, 2.8498, 0.092815],
        [2.48814156, 1.14836059, 3.21395, 0.08165],
        [2.62010372, 1.1049143, 3.35946, 0.078635],
        [2.75155515, 1.06006668, 3.50497, 0.07562],
        [2.88245693, 1.01371411, 3.237995, 0.08473],
        [3.01283929, 0.96594252, 2.97102, 0.09384],
        [3.14278403, 0.91697795, 2.797015, 0.100055],
        [3.27239538, 0.86710616, 2.62301, 0.10627],
        [3.40178871, 0.81664194, 2.497385, 0.11189],
        [3.52534292, 0.76798582, 2.37176, 0.11751],
        [3.64882806, 0.72098717, 2.27461, 0.12252],
        [3.77225054, 0.67657779, 2.17746, 0.12753],
        [3.89565744, 0.63576533, 2.05692, 0.13501],
        [4.01912628, 0.59935302, 1.93638, 0.14249],
        [4.14273194, 0.56802807, 1.93638, 0.1419],
        [4.26652265, 0.54240254, 1.93638, 0.14131],
        [4.390508, 0.52303129, 1.93638, 0.140645],
        [4.5146562, 0.51041311, 1.93638, 0.13998],
        [4.63889641, 0.50498126, 1.93638, 0.13931],
        [4.76312271, 0.50708933, 1.93638, 0.13864],
        [4.88719857, 0.51699788, 1.93638, 0.13805],
        [5.01096146, 0.53486531, 1.93638, 0.13746],
        [5.13422789, 0.56074489, 1.93638, 0.137045],
        [5.25679889, 0.59458804, 1.93638, 0.13663],
        [5.3784661, 0.63625275, 1.963645, 0.13462],
        [5.49901791, 0.68551547, 1.99091, 0.13261],
        [5.61824556, 0.74208505, 2.01696, 0.13113],
        [5.73594876, 0.8056172, 2.04301, 0.12965],
        [5.85194051, 0.87572882, 2.06793, 0.128605],
        [5.96605088, 0.95201138, 2.09285, 0.12756],
        [6.0781297, 1.03404317, 2.12232, 0.126545],
        [6.18804798, 1.12140005, 2.15179, 0.12553],
        [6.29569809, 1.21366463, 2.18424, 0.1246],
        [6.40099292, 1.31043383, 2.21669, 0.12367],
        [6.50386394, 1.41132475, 2.25317, 0.12272],
        [6.60425847, 1.51597899, 2.28965, 0.12177],
        [6.70213638, 1.62406543, 2.335065, 0.12051],
        [6.79746619, 1.73528168, 2.38048, 0.11925],
        [6.89022097, 1.84935431, 2.430755, 0.117865],
        [6.980374, 1.96603801, 2.48103, 0.11648],
        [7.06789434, 2.08511385, 2.54234, 0.11467],
        [7.15274241, 2.20638682, 2.60365, 0.11286],
        [7.23486553, 2.32968268, 2.60826, 0.113455],
        [7.31419351, 2.4548443, 2.61287, 0.11405],
        [7.39063436, 2.58172754, 2.475375, 0.12144],
        [7.46407004, 2.71019664, 2.33788, 0.12883],
        [7.5343525, 2.84011927, 2.23473, 0.13553],
        [7.60129994, 2.9713612, 2.13158, 0.14223],
        [7.6646937, 3.10378064, 2.050635, 0.14829],
        [7.72427579, 3.2372224, 1.96969, 0.15435],
        [7.77974745, 3.37151184, 1.903695, 0.159935],
        [7.83076905, 3.50644891, 1.8377, 0.16552],
        [7.87696157, 3.64180249, 1.78245, 0.17064],
        [7.91791, 3.77730515, 1.7272, 0.17576],
        [7.9531689, 3.91264892, 1.67936, 0.179975],
        [7.98227014, 4.0474821, 1.63152, 0.18419],
        [8.00473301, 4.18140776, 1.588265, 0.187325],
        [8.02007634, 4.31398396, 1.54501, 0.19046],
        [8.02783246, 4.44472614, 1.4693, 0.19776],
        [8.02756242, 4.57311169, 1.39359, 0.20506],
        [8.01887186, 4.69858671, 1.39359, 0.201645],
        [8.00142678, 4.82057488, 1.39359, 0.19823],
        [7.97496831, 4.93848799, 1.39359, 0.194875],
        [7.9393258, 5.05173771, 1.39359, 0.19152],
        [7.89442733, 5.15974804, 1.39359, 0.188555],
        [7.84030706, 5.26196766, 1.39359, 0.18559],
        [7.77710908, 5.35788145, 1.39359, 0.183265],
        [7.7050874, 5.44702069, 1.39359, 0.18094],
        [7.62460227, 5.52897101, 1.39359, 0.17943],
        [7.536113, 5.60337802, 1.39359, 0.17792],
        [7.44016781, 5.66995014, 1.41467, 0.174655],
        [7.33739134, 5.72845868, 1.43575, 0.17139],
        [7.22847048, 5.77873548, 1.45523, 0.16938],
        [7.11413941, 5.82066844, 1.47471, 0.16737],
        [6.99516423, 5.85419554, 1.497445, 0.16569],
        [6.87232807, 5.8792982, 1.52018, 0.16401],
        [6.74641682, 5.89599457, 1.54315, 0.16271],
        [6.61820582, 5.90433369, 1.56612, 0.16141],
        [6.48844757, 5.90439106, 1.596905, 0.159545],
        [6.35786039, 5.89626615, 1.62769, 0.15768],
        [6.22711782, 5.88008231, 1.662375, 0.155595],
        [6.09683868, 5.85598899, 1.69706, 0.15351],
        [5.96757762, 5.82416624, 1.738965, 0.150945],
        [5.83981608, 5.78483111, 1.78087, 0.14838],
        [5.71395379, 5.73824529, 1.832985, 0.145225],
        [5.59030113, 5.68472331, 1.8851, 0.14207],
        [5.46907261, 5.62464036, 1.949255, 0.13839],
        [5.35038212, 5.55843893, 2.01341, 0.13471],
        [5.23424046, 5.48663315, 2.09467, 0.130435],
        [5.12055594, 5.40981015, 2.17593, 0.12616],
        [5.00913994, 5.32862503, 2.28189, 0.12121],
        [4.89972666, 5.24377382, 2.38785, 0.11626],
        [4.79201344, 5.15593375, 2.186985, 0.12874],
        [4.6857631, 5.06560545, 1.98612, 0.14122],
        [4.5807927, 4.97315917, 1.85338, 0.152835],
        [4.48081878, 4.88243357, 1.72064, 0.16445],
        [4.37959748, 4.79420328, 1.72064, 0.16502],
        [4.27684354, 4.70906838, 1.72064, 0.16559],
        [4.17224941, 4.62769566, 1.72064, 0.16074],
        [4.06553767, 4.55071821, 1.72064, 0.15589],
        [3.95644174, 4.47877854, 1.72064, 0.15523],
        [3.84474817, 4.41244104, 1.72064, 0.15457],
        [3.73031118, 4.35215634, 1.72064, 0.15353],
        [3.61306059, 4.29823493, 1.72064, 0.15249],
        [3.49300409, 4.25082828, 1.72064, 0.1515],
        [3.37022468, 4.20991786, 1.72064, 0.15051],
        [3.24487511, 4.17530971, 1.84541, 0.140615],
        [3.1171707, 4.14663361, 1.97018, 0.13072],
        [2.98738247, 4.12334422, 2.08479, 0.12419],
        [2.85585139, 4.10466958, 2.1994, 0.11766],
        [2.72282052, 4.09005797, 2.12722, 0.122425],
        [2.58852158, 4.07896026, 2.05504, 0.12719],
        [2.45315288, 4.07089104, 1.926685, 0.137095],
        [2.31675601, 4.06579649, 1.79833, 0.147],
        [2.18425079, 4.05787239, 1.70739, 0.15607],
        [2.05356546, 4.04652377, 1.61645, 0.16514],
        [1.9251617, 4.03105575, 1.54612, 0.173545],
        [1.79950825, 4.01084184, 1.47579, 0.18195],
        [1.67721911, 3.98511327, 1.387895, 0.192675],
        [1.55873527, 3.9534865, 1.3, 0.2034],
        [1.44466638, 3.91541352, 1.3, 0.20018],
        [1.33564671, 3.8704324, 1.3, 0.19696],
        [1.23222981, 3.81830074, 1.3, 0.19274],
        [1.13519251, 3.7586213, 1.3, 0.18852],
        [1.04519953, 3.69128277, 1.3, 0.18369],
        [0.96285535, 3.61637202, 1.3, 0.17886],
        [0.88870748, 3.5341512, 1.3, 0.173865],
        [0.82324005, 3.44504611, 1.3, 0.16887],
        [0.76686864, 3.34963169, 1.3, 0.164135],
        [0.71993756, 3.24861421, 1.3, 0.1594],
        [0.68271928, 3.1428114, 1.30893, 0.153905],
        [0.65541543, 3.03313123, 1.31786, 0.14841],
        [0.63815905, 2.92055024, 1.32535, 0.144805],
        [0.63069109, 2.80611932, 1.33284, 0.1412],
    ]

    # Get closest indexes for racing line (and distances to all points on racing line)
    closest_index, second_closest_index = closest_2_racing_points_index(
        racing_track, [P.x, P.y]
    )

    # Get optimal [x, y, speed, time] for closest and second closest index
    optimals = racing_track[closest_index]
    optimals_second = racing_track[second_closest_index]

    G.reward = 0.001

    ## Incentive for finishing the lap in less steps ##
    if P.progress == 100:
        return max(
            1e-3,
            (
                -SETTINGS.REWARD_FOR_FASTEST_TIME
                / (15 * (SETTINGS.STANDARD_TIME - SETTINGS.FASTEST_TIME))
            )
            * (P.steps - SETTINGS.STANDARD_TIME * 15),
        )

    ## Reward if car goes close to optimal racing line ##
    dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [P.x, P.y])
    G.distance_reward = max(1e-3, 1 - (dist / (P.track_width * 0.8)))
    G.reward += G.distance_reward * SETTINGS.DISTANCE_MULTIPLIER

    G.reward += SETTINGS.PROGRESS_MULTIPLIER

    if P.all_wheels_on_track == False:
        G.reward = 0.001
        if SETTINGS.verbose:
            print(f"OFF TRACK")

    # Zero reward if obviously wrong direction (e.g. spin)
    direction_diff = racing_direction_diff(
        optimals[0:2], optimals_second[0:2], [P.x, P.y], P.heading
    )
    if direction_diff > 30:
        G.reward = 0.001
        if SETTINGS.verbose:
            print(f"WRONG DIRECTION: {direction_diff:.1f}")

    return float(G.reward)


def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
    # Calculate the distances between 2 closest racing points
    a = abs(
        dist_2_points(
            x1=closest_coords[0],
            x2=second_closest_coords[0],
            y1=closest_coords[1],
            y2=second_closest_coords[1],
        )
    )

    # Distances between car and closest and second closest racing point
    b = abs(
        dist_2_points(
            x1=car_coords[0],
            x2=closest_coords[0],
            y1=car_coords[1],
            y2=closest_coords[1],
        )
    )
    c = abs(
        dist_2_points(
            x1=car_coords[0],
            x2=second_closest_coords[0],
            y1=car_coords[1],
            y2=second_closest_coords[1],
        )
    )

    # Calculate distance between car and racing line (goes through 2 closest racing points)
    # try-except in case a=0 (rare bug in DeepRacer)
    try:
        distance = abs(
            -(a**4)
            + 2 * (a**2) * (b**2)
            + 2 * (a**2) * (c**2)
            - (b**4)
            + 2 * (b**2) * (c**2)
            - (c**4)
        ) ** 0.5 / (2 * a)
    except:
        distance = b

    return distance


def dist_2_points(x1, x2, y1, y2):
    return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5


def closest_2_racing_points_index(racing_coords, car_coords):
    # Calculate all distances to racing points
    distances = []
    for i in range(len(racing_coords)):
        distance = dist_2_points(
            x1=racing_coords[i][0],
            x2=car_coords[0],
            y1=racing_coords[i][1],
            y2=car_coords[1],
        )
        distances.append(distance)

    # Get index of the closest racing point
    closest_index = distances.index(min(distances))

    # Get index of the second closest racing point
    distances_no_closest = distances.copy()
    distances_no_closest[closest_index] = 999
    second_closest_index = distances_no_closest.index(min(distances_no_closest))

    return [closest_index, second_closest_index]

    # Calculate which one of the closest racing points is the next one and which one the previous one


def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
    # Virtually set the car more into the heading direction
    heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
    new_car_coords = [
        car_coords[0] + heading_vector[0],
        car_coords[1] + heading_vector[1],
    ]

    # Calculate distance from new car coords to 2 closest racing points
    distance_closest_coords_new = dist_2_points(
        x1=new_car_coords[0],
        x2=closest_coords[0],
        y1=new_car_coords[1],
        y2=closest_coords[1],
    )
    distance_second_closest_coords_new = dist_2_points(
        x1=new_car_coords[0],
        x2=second_closest_coords[0],
        y1=new_car_coords[1],
        y2=second_closest_coords[1],
    )

    if distance_closest_coords_new <= distance_second_closest_coords_new:
        next_point_coords = closest_coords
        prev_point_coords = second_closest_coords
    else:
        next_point_coords = second_closest_coords
        prev_point_coords = closest_coords

    return [next_point_coords, prev_point_coords]


def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
    # Calculate the direction of the center line based on the closest waypoints
    next_point, prev_point = next_prev_racing_point(
        closest_coords, second_closest_coords, car_coords, heading
    )

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(
        next_point[1] - prev_point[1], next_point[0] - prev_point[0]
    )

    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    return direction_diff


# Gives back indexes that lie between start and end index of a cyclical list
# (start index is included, end index is not)
def indexes_cyclical(start, end, array_len):
    if end < start:
        end += array_len
    return [index % array_len for index in range(start, end)]


def read_params(params):
    P.all_wheels_on_track = params["all_wheels_on_track"]
    P.x = params["x"]
    P.y = params["y"]
    P.distance_from_center = params["distance_from_center"]
    P.is_left_of_center = params["is_left_of_center"]
    P.heading = params["heading"]
    P.progress = params["progress"]
    P.steps = params["steps"]
    P.speed = params["speed"]
    P.steering_angle = params["steering_angle"]
    P.track_width = params["track_width"]
    P.waypoints = params["waypoints"]
    P.closest_waypoints = params["closest_waypoints"]
    P.is_offtrack = params["is_offtrack"]
