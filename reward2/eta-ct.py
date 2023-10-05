import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

VERBOSE = False

ENABLED_DISTANCE_REWARD = True
ENABLED_SPEED_REWARD = True
ENABLED_STEPS_REWARD = True
ENABLED_FINISH_REWARD = False

REWARD_FOR_FASTEST_TIME = 50
STANDARD_TIME = 12  # seconds (time that is easily done by model)
FASTEST_TIME = 9  # seconds (best time of 1st place on the track)


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):
        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def get_distance(coor1, coor2):
            return math.sqrt(
                (coor1[0] - coor2[0]) * (coor1[0] - coor2[0])
                + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1])
            )

        def get_radians(coor1, coor2):
            return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))

        def get_degrees(coor1, coor2):
            return math.degrees(get_radians(coor1, coor2))

        def get_diff_radians(angle1, angle2):
            diff = (angle1 - angle2) % (2.0 * math.pi)

            if diff >= math.pi:
                diff -= 2.0 * math.pi

            return diff

        def get_diff_degrees(angle1, angle2):
            return math.degrees(get_diff_radians(angle1, angle2))

        def up_sample(waypoints, factor=20):
            p = waypoints
            n = len(p)

            return [
                [
                    i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
                    i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],
                ]
                for j in range(n)
                for i in range(factor)
            ]

        def get_distance_list(car, waypoints):
            dist_list = []
            min_dist = float("inf")
            min_idx = -1

            for i, waypoint in enumerate(waypoints):
                dist = get_distance(car, waypoint)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i
                dist_list.append(dist)

            return dist_list, min_dist, min_idx, len(waypoints)

        def detect_bot(params):
            car = [params["x"], params["y"]]

            heading = math.radians(params["heading"])
            track_width = params["track_width"]
            is_reversed = params["is_reversed"]

            objects_location = params["objects_location"]
            objects_left_of_center = params["objects_left_of_center"]

            warned = False
            is_inner = False

            bot_idx = -1
            bot_dist = float("inf")

            for i, location in enumerate(objects_location):
                dist = get_distance(car, location)

                angle = get_radians(car, location)

                diff = abs(get_diff_degrees(heading, angle))

                if dist < track_width and diff < 120:
                    warned = True

                    if dist < bot_dist:
                        bot_idx = i
                        bot_dist = dist

            if warned:
                if is_reversed:
                    if objects_left_of_center[bot_idx] == False:
                        is_inner = True
                else:
                    if objects_left_of_center[bot_idx]:
                        is_inner = True

            return warned, is_inner, bot_dist

        ################## HELPER FUNCTIONS ###################

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

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(
            closest_coords, second_closest_coords, car_coords, heading
        ):
            # Virtually set the car more into the heading direction
            heading_vector = [
                math.cos(math.radians(heading)),
                math.sin(math.radians(heading)),
            ]
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

        def racing_direction_diff(
            closest_coords, second_closest_coords, car_coords, heading
        ):
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

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):
            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list)
            )

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

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [
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

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        # all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        # distance_from_center = params["distance_from_center"]
        # is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        # steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        # waypoints = params["waypoints"]
        # closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]
        is_reversed = params["is_reversed"]

        # closest_objects = params["closest_objects"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        track = racing_track

        # if closest_objects:
        #     warned, is_inner, _ = detect_bot(params)

        #     if warned:
        #         if is_inner:
        #             track = outer_track
        #         else:
        #             track = inner_track

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            track, [x, y]
        )

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = track[closest_index]
        optimals_second = track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        MIN_REWARD = 1e-2

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 3
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MIN_REWARD, 1 - (dist / (track_width * 0.5)))
        if ENABLED_DISTANCE_REWARD:
            reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        if ENABLED_SPEED_REWARD:
            reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list
        )
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                MIN_REWARD,
                (
                    -REWARD_PER_STEP_FOR_FASTEST_TIME
                    * (FASTEST_TIME)
                    / (STANDARD_TIME - FASTEST_TIME)
                )
                * (steps_prediction - (STANDARD_TIME * 15 + 1)),
            )
            steps_reward = min(
                REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction
            )
        except:
            steps_reward = 0
        if ENABLED_STEPS_REWARD:
            reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading
        )
        if direction_diff > 30:
            reward = MIN_REWARD
        else:
            reward += 1.1 - (direction_diff / 30)

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = MIN_REWARD

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        # REWARD_FOR_FASTEST_TIME = 300
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        if progress > 99.8:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME)))
                * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        if ENABLED_FINISH_REWARD:
            reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = MIN_REWARD

        ####################### VERBOSE #######################
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward(
    verbose=VERBOSE
)  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)



# test features

def get_test_params():
    return {
        "x": 0.7,
        "y": 1.05,
        "speed": 2.0,
        "heading": 160.0,
        "track_width": 0.45,
        "is_reversed": False,
        "steering_angle": 0.0,
        "steps": 200,
        "progress": 99.9,
        "is_offtrack": False,
        "waypoints": [
            [0.75, -0.7],
            [1.0, 0.0],
            [0.7, 0.52],
            [0.58, 0.7],
            [0.48, 0.8],
            [0.15, 0.95],
            [-0.1, 1.0],
            [-0.7, 0.75],
            [-0.9, 0.25],
            [-0.9, -0.55],
        ],
    }


def test_reward():
    params = get_test_params()

    reward = reward_function(params)

    print("test_reward: {}".format(reward))


if VERBOSE:
    test_reward()
