import math


class SETTINGS:
    debug = False
    verbose = True
    STAGE = 1
    REWARD_PER_STEP_FOR_FASTEST_TIME = 1
    REWARD_FOR_FASTEST_TIME = 200  # should be adapted to track length and other rewards. finish_reward = max(1e-3, (-self.REWARD_FOR_FASTEST_TIME / (15*(self.STANDARD_TIME - self.FASTEST_TIME)))*(steps-self.STANDARD_TIME*15))
    MIN_DIST_CLOSING_BONUS = 0.5


class TRACK_INFO:
    STANDARD_TIME = 11.3  # seconds (time that is easily done by model)
    FASTEST_TIME = 8.7  # seconds (best time of 1st place on the track)
    MIN_SPEED = 1.7
    MAX_SPEED = 4.0
    #################### RACING LINE ######################
    # Optimal racing line for the Spain track
    # Each row: [x,y,speed,timeFromPreviousPoint]
    racing_line = [
        [-3.70671, -0.00069, 1.76286, 0.10456],
        [-3.74957, -0.17901, 1.78058, 0.103],
        [-3.76502, -0.36347, 1.80049, 0.10281],
        [-3.75216, -0.55237, 1.82697, 0.10363],
        [-3.70951, -0.74374, 1.85619, 0.10563],
        [-3.63487, -0.935, 1.88195, 0.10909],
        [-3.52554, -1.12231, 1.9064, 0.11377],
        [-3.37913, -1.29984, 1.92611, 0.11947],
        [-3.19534, -1.45904, 1.94357, 0.12511],
        [-2.97798, -1.58974, 2.73434, 0.09276],
        [-2.74543, -1.70255, 3.25047, 0.07952],
        [-2.50243, -1.8016, 4.0, 0.0656],
        [-2.25325, -1.89197, 4.0, 0.06626],
        [-1.98987, -1.98346, 3.88518, 0.07176],
        [-1.72714, -2.07653, 3.43009, 0.08126],
        [-1.46471, -2.17036, 3.10153, 0.08986],
        [-1.20251, -2.26186, 2.84745, 0.09753],
        [-0.94016, -2.34729, 2.5322, 0.10896],
        [-0.67721, -2.42304, 2.5322, 0.10806],
        [-0.41349, -2.48569, 2.5322, 0.10705],
        [-0.14904, -2.53195, 2.5322, 0.10602],
        [0.1158, -2.55861, 2.5322, 0.10512],
        [0.38035, -2.56246, 2.5322, 0.10448],
        [0.64322, -2.53782, 2.60349, 0.10141],
        [0.90303, -2.48631, 2.67163, 0.09914],
        [1.15858, -2.40911, 2.73681, 0.09754],
        [1.40868, -2.30709, 2.81388, 0.09599],
        [1.65216, -2.18114, 2.89874, 0.09457],
        [1.88784, -2.0322, 2.99416, 0.09312],
        [2.11454, -1.86134, 3.11293, 0.09119],
        [2.33119, -1.67006, 3.24443, 0.08908],
        [2.53678, -1.46013, 3.40478, 0.0863],
        [2.7306, -1.23375, 3.41684, 0.08722],
        [2.91216, -0.99345, 3.05722, 0.09852],
        [3.08125, -0.74181, 2.78745, 0.10877],
        [3.23809, -0.48136, 2.57575, 0.11803],
        [3.38341, -0.21415, 2.40314, 0.12657],
        [3.51457, 0.05962, 2.25865, 0.1344],
        [3.62575, 0.33881, 2.13353, 0.14085],
        [3.71147, 0.62031, 2.02039, 0.14565],
        [3.76762, 0.9005, 1.82239, 0.15681],
        [3.79156, 1.17572, 1.82239, 0.15159],
        [3.78181, 1.44244, 1.82239, 0.14646],
        [3.73766, 1.69728, 1.82239, 0.14192],
        [3.65866, 1.93674, 1.82239, 0.13836],
        [3.54431, 2.15674, 1.82239, 0.13606],
        [3.39063, 2.34894, 1.87752, 0.13107],
        [3.20392, 2.51037, 1.92847, 0.12799],
        [2.98981, 2.63811, 1.98793, 0.12542],
        [2.75425, 2.72983, 2.04801, 0.12343],
        [2.50334, 2.78384, 2.12852, 0.12058],
        [2.24332, 2.80008, 2.21923, 0.11739],
        [1.97986, 2.77967, 2.32883, 0.11347],
        [1.7177, 2.72491, 2.46513, 0.10864],
        [1.46042, 2.63904, 2.63292, 0.10301],
        [1.21037, 2.52577, 2.84544, 0.09647],
        [0.96872, 2.38911, 3.12258, 0.08891],
        [0.73562, 2.23314, 2.59723, 0.10799],
        [0.51046, 2.06177, 2.25006, 0.12575],
        [0.29193, 1.87895, 2.25006, 0.12663],
        [0.09152, 1.70065, 2.25006, 0.11921],
        [-0.11327, 1.53096, 2.25006, 0.1182],
        [-0.3263, 1.37778, 2.25006, 0.11661],
        [-0.55076, 1.24863, 2.25006, 0.11509],
        [-0.78913, 1.1511, 2.57639, 0.09996],
        [-1.03737, 1.07798, 2.87613, 0.08998],
        [-1.29333, 1.02501, 2.68736, 0.09726],
        [-1.55506, 0.98783, 2.35167, 0.11241],
        [-1.82072, 0.96175, 2.11381, 0.12628],
        [-2.08849, 0.9417, 1.92988, 0.13914],
        [-2.3512, 0.91172, 1.7, 0.15554],
        [-2.60313, 0.86596, 1.7, 0.15062],
        [-2.83928, 0.80044, 1.7, 0.14416],
        [-3.05488, 0.71337, 1.7, 0.13677],
        [-3.24578, 0.60497, 1.7, 0.12913],
        [-3.40863, 0.47684, 1.7, 0.12189],
        [-3.53778, 0.32997, 1.72336, 0.11349],
        [-3.63645, 0.16971, 1.74295, 0.10798],
    ]


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
    closest_waypoints = [0, 1]
    is_offtrack = None


# Global
class G:
    direction_diff = None
    optimals = None
    optimals_second = None
    route_direction = None
    sigma_speed = None
    normalized_distance_from_route = None
    dist_to_racing_line = None
    # intermediate_progress = [0] * 71
    next_index = 1
    intermediate_progress_bonus = None
    projected_time = None
    reward_prediction = None
    steps_prediction = None


class STATE:
    prev_speed = None
    prev_steering_angle = None
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None
    first_racingpoint_index = None


class REWARDS:
    final = 0.001
    speed = 0
    distance = 0
    progress = 0
    immediate = 0
    finish = 0


class OPTIMAL:
    speed = 0


def read_params(params):
    ################## INPUT PARAMETERS ###################
    # Read all input parameters
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


def reset_global():
    direction_diff = None
    optimals = [2]
    optimals_second = [2]
    route_direction = None
    sigma_speed = None
    normalized_distance_from_route = None
    dist_to_racing_line = None
    # intermediate_progress = [0] * 71
    next_index = None
    intermediate_progress_bonus = None
    projected_time = None
    reward_prediction = None
    steps_prediction = None


def get_progress_reward(closest_index):
    times_list = [row[3] for row in TRACK_INFO.racing_line]
    G.projected_time = get_projected_time(
        STATE.first_racingpoint_index, closest_index, P.steps, times_list
    )
    G.steps_prediction = G.projected_time * 15 + 1
    G.reward_prediction = max(
        1e-3,
        (
            -SETTINGS.REWARD_PER_STEP_FOR_FASTEST_TIME
            * (TRACK_INFO.FASTEST_TIME)
            / (TRACK_INFO.STANDARD_TIME - TRACK_INFO.FASTEST_TIME)
        )
        * (G.steps_prediction - (TRACK_INFO.STANDARD_TIME * 15 + 1)),
    )
    return min(
        SETTINGS.REWARD_PER_STEP_FOR_FASTEST_TIME,
        G.reward_prediction / G.steps_prediction,
    )


def get_distance_reward():
    d = max(1e-3, 1 - (abs(G.dist_to_racing_line) / (P.track_width * 0.8)))
    # Reward reducing distance to the race line
    # distance_reduction_bonus = 1
    if STATE.prev_normalized_distance_from_route is not None:
        if abs(STATE.prev_normalized_distance_from_route) > abs(G.dist_to_racing_line):
            print(
                f"BONUS: STATE.prev_normalized_distance_from_route > G.dist_to_racing_line: {STATE.prev_normalized_distance_from_route:.1f} > {G.dist_to_racing_line:.1f}"
            )
            return max(d, SETTINGS.MIN_DIST_CLOSING_BONUS)

    # return 1
    # distance reward is value of the standard normal scaled back to 1.
    # Hence the 1/2*pi*sigma term is cancelled out
    #    sigma=abs(normalized_route_distance_from_inner_border / 4)
    #  return math.exp(-0.5*abs(normalized_car_distance_from_route)**2/G.sigma**2)
    return d


def get_heading_reward():
    if abs(G.direction_diff) <= 20:
        h = math.cos(abs(G.direction_diff) * (math.pi / 180)) ** 4
    h = math.cos(abs(G.direction_diff) * (math.pi / 180)) ** 10
    if STATE.prev_direction_diff is not None:
        if abs(G.direction_diff) < abs(STATE.prev_direction_diff):
            h = max(h, 0.2)
            if SETTINGS.verbose:
                print(f"BONUS GETTING RIGHT DIR")
    return h


def get_speed_reward():
    # return (P.speed - 1.3) / (4.0 - 1.3)
    return math.exp(-0.5 * abs(P.speed - OPTIMAL.speed) ** 2 / G.sigma_speed**2)


def get_final_reward():
    if P.is_offtrack:
        return 0.001
        if SETTINGS.verbose:
            print(f"OFF TRACK")
    #    return max(REWARDS.immediate + G.intermediate_progress_bonus, 1e-3)
    return max(REWARDS.immediate + REWARDS.finish, 1e-3)


def get_immediate_reward():
    # if SETTINGS.STAGE == 1:
    #     lc = (REWARDS.distance) ** 2 + (REWARDS.distance)
    # elif SETTINGS.STAGE == 2:
    #     lc = (REWARDS.speed + REWARDS.distance) ** 2 + ( REWARDS.speed * REWARDS.distance)
    # else:
    lc = REWARDS.distance
    # lc = (REWARDS.speed + REWARDS.distance + REWARDS.heading) ** 2 + ( REWARDS.speed * REWARDS.distance * REWARDS.heading)
    # lc = (REWARDS.distance) ** 2 + (REWARDS.distance)

    ## Stage 1 Checks

    if SETTINGS.STAGE < 2:
        return max(lc, 1e-3)

    ###############################################################
    ## Stage 2 Checks
    ###############################################################

    if is_right_turn_section() and P.steering_angle > 0:
        if SETTINGS.verbose:
            print(f"!!! SHOULD NOT MAKE LEFT TURN IN RIGHT TURN SECTION")
        return 1e-3

    if is_left_turn_section() and P.steering_angle < -5:
        if SETTINGS.verbose:
            print(f"!!! SHOULD NOT MAKE RIGHT TURN IN LEFT TURN SECTION")
        return 1e-3

    if is_straight_section() and P.steps > 10 and abs(P.steering_angle > 15):
        if SETTINGS.verbose:
            print(f"!!! SHOULD NOT MAKE SHARP TURN IN STRIAGHT SECTION")
        return 1e-3

    if is_first_left_turn_section():
        if P.speed - OPTIMAL.speed > 1:
            if SETTINGS.verbose:
                print(f"!!! TOO FAST")
            return 1e-3

    # avoid sharp turn if previous speed is fast
    # if STATE.prev_speed > 2.3 and abs(P.steering_angle > 20):
    #     if SETTINGS.verbose:
    #         print(f"!!! SHOULD NOT MAKE SHARP TURN IF PREVIOUS SPEED IS TOO FAST")
    #     return 1e-3

    # Zero reward if obviously wrong direction (e.g. spin) and it is getting worst
    # below cannot tell diff is right or left
    # P.direction_diff = racing_direction_diff(P.optimals[0:2], P.optimals_second[0:2], [P.x, P.y], P.heading)
    if STATE.prev_direction_diff is not None:
        if abs(G.direction_diff) > 30 and (
            abs(G.direction_diff) > abs(STATE.prev_direction_diff)
        ):
            if SETTINGS.verbose:
                print(
                    f"!!! FAR AWAY FROM DIRECTION AND GETTING WORST: {G.direction_diff:.1f}, prev: {STATE.prev_direction_diff}"
                )
            return 1e-3

    if OPTIMAL.speed - P.speed > 2 and is_straight_section():
        if SETTINGS.verbose:
            print(f"!!! TOO SLOW")
        return 1e-3

    if SETTINGS.STAGE < 3:
        return max(lc, 1e-3)

    ###############################################################
    ## Stage 3 Checks
    ###############################################################

    return max(lc, 1e-3)


def is_right_turn_section():
    return (P.closest_waypoints[0] > 25 or P.closest_waypoints[1] > 25) and (
        P.closest_waypoints[0] < 34 or P.closest_waypoints[1] < 34
    )


def is_first_left_turn_section():
    return (P.closest_waypoints[0] > 10 or P.closest_waypoints[1] > 10) and (
        P.closest_waypoints[0] < 23 or P.closest_waypoints[1] < 23
    )


def is_second_left_turn_section():
    return (P.closest_waypoints[0] > 40 or P.closest_waypoints[1] > 40) and (
        P.closest_waypoints[0] < 43 or P.closest_waypoints[1] < 43
    )


def is_left_turn_section():
    return (
        (P.closest_waypoints[0] > 10 or P.closest_waypoints[1] > 10)
        and (P.closest_waypoints[0] < 23 or P.closest_waypoints[1] < 23)
        or (P.closest_waypoints[0] > 40 or P.closest_waypoints[1] > 40)
        and (P.closest_waypoints[0] < 43 or P.closest_waypoints[1] < 43)
        or (P.closest_waypoints[0] > 49 or P.closest_waypoints[1] > 49)
        and (P.closest_waypoints[0] < 52 or P.closest_waypoints[1] < 52)
        or (P.closest_waypoints[0] > 61 or P.closest_waypoints[1] > 61)
        and (P.closest_waypoints[0] < 67 or P.closest_waypoints[1] < 67)
    )


def is_straight_section():
    return (
        (P.closest_waypoints[0] > 68 or P.closest_waypoints[1] > 68)
        or (P.closest_waypoints[0] < 10 or P.closest_waypoints[1] < 10)
        or (P.closest_waypoints[0] > 53 or P.closest_waypoints[1] > 53)
        and (P.closest_waypoints[0] < 60 or P.closest_waypoints[1] < 60)
    )


def init_state():
    STATE.prev_speed = None
    STATE.prev_steering_angle = None
    STATE.prev_direction_diff = None
    STATE.prev_normalized_distance_from_route = None


def print_params():
    if not SETTINGS.verbose:
        return

    if not SETTINGS.verbose:
        return
    if STATE.prev_speed is not None:
        print(
            f"state: sp:{STATE.prev_speed:.1f} st:{STATE.prev_steering_angle:.1f} dd:{STATE.prev_direction_diff:.1f}, dt:{STATE.prev_normalized_distance_from_route:.1f}"
        )
    else:
        print("empty state")

    FINAL_BAR_LENGTH = 10
    SPEED_BAR_LENGTH = 5
    capped_final = min(REWARDS.final, FINAL_BAR_LENGTH)
    print(
        f"st:{SETTINGS.STAGE},r:{REWARDS.final:.2f} {'*' * math.ceil(capped_final*1)}{' ' * math.floor(FINAL_BAR_LENGTH-capped_final*1)}",
        end=" ",
    )
    capped_speed = min(REWARDS.speed, SPEED_BAR_LENGTH)
    normalized_speed = (REWARDS.speed - 1.3) * SPEED_BAR_LENGTH / 4.0
    print(
        f"sr:{REWARDS.speed:.1f} {'*' * math.ceil(normalized_speed)}{' ' * math.floor(SPEED_BAR_LENGTH-normalized_speed)}",
        end=" ",
    )
    print(
        f"dr:{REWARDS.distance:.1f} {'*' * math.ceil(REWARDS.distance*5)}{' ' * math.floor(5-REWARDS.distance*5)}",
        end=" ",
    )
    print(
        f"hr:{REWARDS.heading:.1f} {'*' * math.ceil(REWARDS.heading*5)}{' ' * math.floor(5-REWARDS.heading*5)}",
        end=" ",
    )
    print(f"pr:{REWARDS.progress:.1f}", end=" ")
    speed_bar = (P.speed - 1.3) * 10.0 / (4.0 - 1.3)
    print(
        f'sp:{P.speed:.1f} {"=" * math.ceil(speed_bar)}{" " * math.floor(10 - speed_bar)}',
        end=" ",
    )
    _l = max(0, P.steering_angle / 3)
    print(
        f'sa:{P.steering_angle:5.1f} {" " * math.floor(10 - _l)}{"<" * math.ceil(_l)}',
        end="|",
    )
    _r = max(0, P.steering_angle / -3)
    print(f'{">" * math.ceil(_r)}{" " * math.floor(10 - _r)}', end=" ")
    print(
        f"x:{P.x:.1f}, y:{P.y:.1f}, h:{P.heading:.1f}, mr:{REWARDS.immediate:.1f}, ir:{G.intermediate_progress_bonus:.1f}, os:{OPTIMAL.speed:.1f}, dd:{G.direction_diff:.1f}, rd:{G.route_direction:.1f} ni:{G.next_index}, pt:{G.projected_time:.1f},dt:{G.dist_to_racing_line:.1f}"
    )
    if SETTINGS.debug:
        print(
            f"dc: {P.distance_from_center:.2f}, p:{P.progress:.2f}, st:{P.steps:3.0f}, cw:{P.closest_waypoints}, rd:{G.route_direction:.1f}, aw: {P.all_wheels_on_track}, il: {P.is_left_of_center}, 2ox:{G.optimals_second[0]}, 2oy:{G.optimals_second[1]}"
        )
        print(
            f"ot: {P.is_offtrack}, tw: {P.track_width:.2f}, ni: {G.next_index}, {TRACK_INFO.racing_line[G.next_index]}"
        )
        print(
            f"-{SETTINGS.REWARD_PER_STEP_FOR_FASTEST_TIME} * {TRACK_INFO.FASTEST_TIME} / ({TRACK_INFO.STANDARD_TIME} - {TRACK_INFO.FASTEST_TIME}))*({G.steps_prediction} - ({TRACK_INFO.STANDARD_TIME}*15+1))"
        )
        print(
            f"REWARDS.progress = min({SETTINGS.REWARD_PER_STEP_FOR_FASTEST_TIME}, {G.reward_prediction} / {G.steps_prediction})"
        )

    print(
        f"{{'all_wheels_on_track':{P.all_wheels_on_track},'x':{P.x},'y':{P.y},'distance_from_center':{P.distance_from_center},'is_left_of_center':{P.is_left_of_center},'heading':{P.heading},'progress':{P.progress},'steps':{P.steps},'speed':{P.speed},'steering_angle':{P.steering_angle},'track_width':{P.track_width},'waypoints':self.waypoints ,'closest_waypoints':{P.closest_waypoints},'is_offtrack':{P.is_offtrack}}}"
    )


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
    ob = dist_2_points(
        x1=car_coords[0], x2=closest_coords[0], y1=car_coords[1], y2=closest_coords[1]
    )
    b = abs(ob)
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
        distance = (
            -(a**4)
            + 2 * (a**2) * (b**2)
            + 2 * (a**2) * (c**2)
            - (b**4)
            + 2 * (b**2) * (c**2)
            - (c**4)
        ) ** 0.5 / (2 * a)
    except:
        distance = ob

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


# Gives back indexes that lie between start and end index of a cyclical list
# (start index is included, end index is not)
def indexes_cyclical(start, end, array_len):
    if end < start:
        end += array_len

    return [index % array_len for index in range(start, end)]


# Calculate how long car would take for entire lap, if it continued like it did until now
def get_projected_time(first_index, closest_index, step_count, times_list):
    # Calculate how much time has passed since start
    current_actual_time = (step_count - 1) / 15

    # Calculate which indexes were already passed
    indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

    # Calculate how much time should have passed if car would have followed optimals
    current_expected_time = sum([times_list[i] for i in indexes_traveled])

    # Calculate how long one entire lap takes if car follows optimals
    total_expected_time = sum(times_list)

    # Calculate how long car would take for entire lap, if it continued like it did until now
    try:
        projected_time = (
            current_actual_time / current_expected_time
        ) * total_expected_time
    except:
        projected_time = 9999

    return projected_time


def reward_function(params):
    read_params(params)

    reset_global()

    # Get closest indexes for racing line (and distances to all points on racing line)
    closest_index, second_closest_index = closest_2_racing_points_index(
        TRACK_INFO.racing_line, [P.x, P.y]
    )

    # Save first racingpoint of episode for later
    if STATE.first_racingpoint_index is None:
        STATE.first_racingpoint_index = closest_index

    # Get optimal [x, y, speed, time] for closest and second closest index
    G.optimals = TRACK_INFO.racing_line[closest_index]
    G.optimals_second = TRACK_INFO.racing_line[second_closest_index]

    if max(P.closest_waypoints[0], P.closest_waypoints[1]) < len(
        TRACK_INFO.racing_line
    ):
        G.next_index = min(max(P.closest_waypoints[0], P.closest_waypoints[1]) + 1, 69)
    else:
        G.next_index = 0

    next_point_coords = TRACK_INFO.racing_line[G.next_index]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians between target and current vehicle position
    G.route_direction = math.atan2(
        next_point_coords[1] - P.y, next_point_coords[0] - P.x
    )
    # Convert to degree
    G.route_direction = math.degrees(G.route_direction)
    # Calculate the difference between the track direction and the heading direction of the car
    if (P.heading > 0 and G.route_direction < 0) or (
        P.heading < 0 and G.route_direction > 0
    ):
        G.direction_diff = G.route_direction + P.heading
    else:
        G.direction_diff = G.route_direction - P.heading

    G.dist_to_racing_line = dist_to_racing_line(
        G.optimals[0:2], G.optimals_second[0:2], [P.x, P.y]
    )

    # Reinitialize previous parameters if it is a new episode
    if STATE.prev_steps is None or P.steps < STATE.prev_steps:
        init_state()

    is_heading_in_right_direction = True
    # Check if the speed has dropped
    has_speed_dropped = False
    if STATE.prev_speed is not None:
        if STATE.prev_speed > P.speed:
            has_speed_dropped = True

    # Penalize slowing down without good reason on straight portions
    # if has_speed_dropped and not is_turn_upcoming:
    #     speed_maintain_bonus = min( P.speed / STATE.prev_speed, 1 )
    # Penalize making the heading direction worse
    heading_decrease_bonus = 0
    if STATE.prev_direction_diff is not None and G.direction_diff != 0:
        if is_heading_in_right_direction:
            if abs(STATE.prev_direction_diff / G.direction_diff) > 1:
                heading_decrease_bonus = min(
                    10, abs(STATE.prev_direction_diff / G.direction_diff)
                )
    # has the steering angle changed
    has_steering_angle_changed = False
    if STATE.prev_steering_angle is not None:
        if not (math.isclose(STATE.prev_steering_angle, P.steering_angle)):
            has_steering_angle_changed = True
    steering_angle_maintain_bonus = 1
    # Not changing the steering angle is a good thing if heading in the right direction
    if is_heading_in_right_direction and not has_steering_angle_changed:
        if abs(G.direction_diff) < 10:
            steering_angle_maintain_bonus *= 2
        if abs(G.direction_diff) < 5:
            steering_angle_maintain_bonus *= 2
        if STATE.prev_direction_diff is not None and abs(
            STATE.prev_direction_diff
        ) > abs(G.direction_diff):
            steering_angle_maintain_bonus *= 2

    G.sigma_speed = abs(TRACK_INFO.MAX_SPEED - TRACK_INFO.MIN_SPEED) / 6.0
    OPTIMAL.speed = G.optimals_second[2]

    ## Reward if car goes close to optimal racing line ##
    # G.normalized_distance_from_route = G.dist_to_racing_line
    REWARDS.heading = get_heading_reward()
    REWARDS.distance = get_distance_reward()
    REWARDS.speed = get_speed_reward()
    REWARDS.progress = get_progress_reward(closest_index)
    REWARDS.immediate = get_immediate_reward()

    # Reward for making steady progress
    G.intermediate_progress_bonus = 0
    # REWARDS.progress = 10 * P.progress / P.steps
    # if P.steps <= 5:
    #     REWARDS.progress = 1 #ignore progress in the first 5 steps
    # # Bonus that the agent gets for completing every 10 percent of track
    # # Is exponential in the progress / steps.
    # # exponent increases with an increase in fraction of lap completed
    # pi = int(P.progress//10)
    # if pi != 0 and G.intermediate_progress[ pi ] == 0:
    #     if pi==10: # 100% track completion
    #         G.intermediate_progress_bonus = REWARDS.progress ** 14
    #     else:
    #         G.intermediate_progress_bonus = REWARDS.progress ** (5+0.75*pi)
    # G.intermediate_progress[ pi ] = G.intermediate_progress_bonus

    # REWARDS.progress = min(1, REWARDS.progress)

    ## Incentive for finishing the lap in less steps ##
    if P.progress == 100:
        REWARDS.finish = max(
            1e-3,
            (
                -SETTINGS.REWARD_FOR_FASTEST_TIME
                / (15 * (TRACK_INFO.STANDARD_TIME - TRACK_INFO.FASTEST_TIME))
            )
            * (P.steps - TRACK_INFO.STANDARD_TIME * 15),
        )
        print(f"finish reward:{REWARDS.finish:.1f}")
    else:
        REWARDS.finish = 0

    REWARDS.final = get_final_reward()

    print_params()

    # Before returning reward, update the variables
    STATE.prev_speed = P.speed
    STATE.prev_steering_angle = P.steering_angle
    STATE.prev_direction_diff = G.direction_diff
    STATE.prev_steps = P.steps
    STATE.prev_normalized_distance_from_route = G.dist_to_racing_line

    return float(REWARDS.final)
