import math


class SETTINGS:
    verbose = False
    debug = False
    REWARD_FOR_FASTEST_TIME = 500  # should be adapted to track length and other rewards. finish_reward = max(1e-3, (-self.REWARD_FOR_FASTEST_TIME / (15*(self.STANDARD_TIME - self.FASTEST_TIME)))*(steps-self.STANDARD_TIME*15))
    STANDARD_TIME = 10.5  # seconds (time that is easily done by model)
    FASTEST_TIME = 8.3  # seconds (best time of 1st place on the track)
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
        [-3.70671, -0.00069, 1.86656, 0.09875],
        [-3.74957, -0.17901, 1.88532, 0.09728],
        [-3.76502, -0.36347, 1.9064, 0.0971],
        [-3.75216, -0.55237, 1.93444, 0.09788],
        [-3.70951, -0.74374, 1.96538, 0.09976],
        [-3.63487, -0.935, 1.99265, 0.10303],
        [-3.52554, -1.12231, 2.01854, 0.10745],
        [-3.37913, -1.29984, 2.03941, 0.11283],
        [-3.19534, -1.45904, 2.0579, 0.11816],
        [-2.97798, -1.58974, 2.89518, 0.0876],
        [-2.74543, -1.70255, 3.44167, 0.0751],
        [-2.50243, -1.8016, 4.0, 0.0656],
        [-2.25325, -1.89197, 4.0, 0.06626],
        [-1.98987, -1.98346, 4.0, 0.0697],
        [-1.72714, -2.07653, 3.63186, 0.07675],
        [-1.46471, -2.17036, 3.28397, 0.08487],
        [-1.20251, -2.26186, 3.01494, 0.09211],
        [-0.94016, -2.34729, 2.68115, 0.10291],
        [-0.67721, -2.42304, 2.68115, 0.10206],
        [-0.41349, -2.48569, 2.68115, 0.1011],
        [-0.14904, -2.53195, 2.68115, 0.10013],
        [0.1158, -2.55861, 2.68115, 0.09928],
        [0.38035, -2.56246, 2.68115, 0.09868],
        [0.64322, -2.53782, 2.75664, 0.09578],
        [0.90303, -2.48631, 2.82878, 0.09364],
        [1.15858, -2.40911, 2.8978, 0.09212],
        [1.40868, -2.30709, 2.9794, 0.09066],
        [1.65216, -2.18114, 3.06926, 0.08931],
        [1.88784, -2.0322, 3.17029, 0.08794],
        [2.11454, -1.86134, 3.29605, 0.08613],
        [2.33119, -1.67006, 3.43528, 0.08413],
        [2.53678, -1.46013, 3.60506, 0.08151],
        [2.7306, -1.23375, 3.61783, 0.08237],
        [2.91216, -0.99345, 3.23706, 0.09304],
        [3.08125, -0.74181, 2.95142, 0.10272],
        [3.23809, -0.48136, 2.72727, 0.11147],
        [3.38341, -0.21415, 2.5445, 0.11954],
        [3.51457, 0.05962, 2.39151, 0.12694],
        [3.62575, 0.33881, 2.25903, 0.13302],
        [3.71147, 0.62031, 2.13924, 0.13756],
        [3.76762, 0.9005, 1.92959, 0.1481],
        [3.79156, 1.17572, 1.92959, 0.14317],
        [3.78181, 1.44244, 1.92959, 0.13832],
        [3.73766, 1.69728, 1.92959, 0.13404],
        [3.65866, 1.93674, 1.92959, 0.13068],
        [3.54431, 2.15674, 1.92959, 0.1285],
        [3.39063, 2.34894, 1.98796, 0.12378],
        [3.20392, 2.51037, 2.04191, 0.12088],
        [2.98981, 2.63811, 2.10487, 0.11845],
        [2.75425, 2.72983, 2.16848, 0.11657],
        [2.50334, 2.78384, 2.25372, 0.11388],
        [2.24332, 2.80008, 2.34977, 0.11087],
        [1.97986, 2.77967, 2.46582, 0.10717],
        [1.7177, 2.72491, 2.61014, 0.10261],
        [1.46042, 2.63904, 2.7878, 0.09729],
        [1.21037, 2.52577, 3.01282, 0.09111],
        [0.96872, 2.38911, 3.30626, 0.08397],
        [0.73562, 2.23314, 2.75001, 0.10199],
        [0.51046, 2.06177, 2.38242, 0.11877],
        [0.29193, 1.87895, 2.38242, 0.11959],
        [0.09152, 1.70065, 2.38242, 0.11259],
        [-0.11327, 1.53096, 2.38242, 0.11164],
        [-0.3263, 1.37778, 2.38242, 0.11013],
        [-0.55076, 1.24863, 2.38242, 0.1087],
        [-0.78913, 1.1511, 2.72794, 0.09441],
        [-1.03737, 1.07798, 3.04532, 0.08498],
        [-1.29333, 1.02501, 2.84544, 0.09186],
        [-1.55506, 0.98783, 2.49, 0.10617],
        [-1.82072, 0.96175, 2.23815, 0.11927],
        [-2.08849, 0.9417, 2.0434, 0.13141],
        [-2.3512, 0.91172, 1.8, 0.1469],
        [-2.60313, 0.86596, 1.8, 0.14225],
        [-2.83928, 0.80044, 1.8, 0.13615],
        [-3.05488, 0.71337, 1.8, 0.12917],
        [-3.24578, 0.60497, 1.8, 0.12196],
        [-3.40863, 0.47684, 1.92959, 0.13832],
        [-3.53778, 0.32997, 2.38242, 0.11259],
        [-3.63645, 0.16971, 3.29605, 0.08613],
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
