import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):
        # Import package (needed for heading)
        import math

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

        # Optimal racing line for the reInvent2019_track_ccw.npy
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [
            [-3.70671, -0.00069, 1.34807, 0.13673],
            [-3.74957, -0.17901, 1.36162, 0.13469],
            [-3.76502, -0.36347, 1.37685, 0.13444],
            [-3.75216, -0.55237, 1.3971, 0.13552],
            [-3.70951, -0.74374, 1.41944, 0.13813],
            [-3.63487, -0.935, 1.43914, 0.14265],
            [-3.52554, -1.12231, 1.45783, 0.14877],
            [-3.37913, -1.29984, 1.47291, 0.15623],
            [-3.19534, -1.45904, 1.48626, 0.1636],
            [-2.97798, -1.58974, 2.09096, 0.1213],
            [-2.74543, -1.70255, 2.48565, 0.10398],
            [-2.50243, -1.8016, 3.21395, 0.08165],
            [-2.25325, -1.89197, 3.50497, 0.07562],
            [-1.98987, -1.98346, 2.97102, 0.09384],
            [-1.72714, -2.07653, 2.62301, 0.10627],
            [-1.46471, -2.17036, 2.37176, 0.11751],
            [-1.20251, -2.26186, 2.17746, 0.12753],
            [-0.94016, -2.34729, 1.93638, 0.14249],
            [-0.67721, -2.42304, 1.93638, 0.14131],
            [-0.41349, -2.48569, 1.93638, 0.13998],
            [-0.14904, -2.53195, 1.93638, 0.13864],
            [0.1158, -2.55861, 1.93638, 0.13746],
            [0.38035, -2.56246, 1.93638, 0.13663],
            [0.64322, -2.53782, 1.99091, 0.13261],
            [0.90303, -2.48631, 2.04301, 0.12965],
            [1.15858, -2.40911, 2.09285, 0.12756],
            [1.40868, -2.30709, 2.15179, 0.12553],
            [1.65216, -2.18114, 2.21669, 0.12367],
            [1.88784, -2.0322, 2.28965, 0.12177],
            [2.11454, -1.86134, 2.38048, 0.11925],
            [2.33119, -1.67006, 2.48103, 0.11648],
            [2.53678, -1.46013, 2.60365, 0.11286],
            [2.7306, -1.23375, 2.61287, 0.11405],
            [2.91216, -0.99345, 2.33788, 0.12883],
            [3.08125, -0.74181, 2.13158, 0.14223],
            [3.23809, -0.48136, 1.96969, 0.15435],
            [3.38341, -0.21415, 1.8377, 0.16552],
            [3.51457, 0.05962, 1.7272, 0.17576],
            [3.62575, 0.33881, 1.63152, 0.18419],
            [3.71147, 0.62031, 1.54501, 0.19046],
            [3.76762, 0.9005, 1.39359, 0.20506],
            [3.79156, 1.17572, 1.39359, 0.19823],
            [3.78181, 1.44244, 1.39359, 0.19152],
            [3.73766, 1.69728, 1.39359, 0.18559],
            [3.65866, 1.93674, 1.39359, 0.18094],
            [3.54431, 2.15674, 1.39359, 0.17792],
            [3.39063, 2.34894, 1.43575, 0.17139],
            [3.20392, 2.51037, 1.47471, 0.16737],
            [2.98981, 2.63811, 1.52018, 0.16401],
            [2.75425, 2.72983, 1.56612, 0.16141],
            [2.50334, 2.78384, 1.62769, 0.15768],
            [2.24332, 2.80008, 1.69706, 0.15351],
            [1.97986, 2.77967, 1.78087, 0.14838],
            [1.7177, 2.72491, 1.8851, 0.14207],
            [1.46042, 2.63904, 2.01341, 0.13471],
            [1.21037, 2.52577, 2.17593, 0.12616],
            [0.96872, 2.38911, 2.38785, 0.11626],
            [0.73562, 2.23314, 1.98612, 0.14122],
            [0.51046, 2.06177, 1.72064, 0.16445],
            [0.29193, 1.87895, 1.72064, 0.16559],
            [0.09152, 1.70065, 1.72064, 0.15589],
            [-0.11327, 1.53096, 1.72064, 0.15457],
            [-0.3263, 1.37778, 1.72064, 0.15249],
            [-0.55076, 1.24863, 1.72064, 0.15051],
            [-0.78913, 1.1511, 1.97018, 0.13072],
            [-1.03737, 1.07798, 2.1994, 0.11766],
            [-1.29333, 1.02501, 2.05504, 0.12719],
            [-1.55506, 0.98783, 1.79833, 0.147],
            [-1.82072, 0.96175, 1.61645, 0.16514],
            [-2.08849, 0.9417, 1.47579, 0.18195],
            [-2.3512, 0.91172, 1.3, 0.2034],
            [-2.60313, 0.86596, 1.3, 0.19696],
            [-2.83928, 0.80044, 1.3, 0.18852],
            [-3.05488, 0.71337, 1.3, 0.17886],
            [-3.24578, 0.60497, 1.3, 0.16887],
            [-3.40863, 0.47684, 1.3, 0.1594],
            [-3.53778, 0.32997, 1.31786, 0.14841],
            [-3.63645, 0.16971, 1.33284, 0.1412],
            [-3.70671, -0.00069, 1.34807, 0.13673],
        ]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        distance_from_center = params["distance_from_center"]
        is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        waypoints = params["waypoints"]
        closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y]
        )

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list
        )
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                1e-3,
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
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading
        )
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = (
            1500  # should be adapted to track length and other rewards
        )
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(
                1e-3,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME)))
                * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

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
    verbose=True
)  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
