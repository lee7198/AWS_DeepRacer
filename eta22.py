import math


class SETTINGS:
    verbose = False
    debug = False
    REWARD_FOR_FASTEST_TIME = 500  # should be adapted to track length and other rewards. finish_reward = max(1e-3, (-self.REWARD_FOR_FASTEST_TIME / (15*(self.STANDARD_TIME - self.FASTEST_TIME)))*(steps-self.STANDARD_TIME*15))
    STANDARD_TIME = 12.5  # seconds (time that is easily done by model)
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
        [0.3078780025243759, 2.830607533454895],
        [0.36493836892326514, 2.6852595333097193],
        [0.4749086754893587, 2.552301440887286],
        [0.5918986669155039, 2.4282316441145317],
        [0.6518188164345089, 2.296265146372148],
        [0.748989451556437, 2.1769059670274857],
        [0.7876174853987443, 2.0534595194612453],
        [0.8489267458216241, 1.9437423243365695],
        [0.9336845462013614, 1.8534063051393161],
        [1.0189016532883173, 1.7732614181923423],
        [1.0852919998183526, 1.7133505218073015],
        [1.1604107616282158, 1.687309938406142],
        [1.2211279006370224, 1.6623731593465092],
        [1.2785974603940917, 1.6167899266463606],
        [1.3546779294148528, 1.5791562992123938],
        [1.4753073340864251, 1.5243748725788473],
        [1.6122521298336336, 1.4724106495245024],
        [1.7621995862766335, 1.4127917567161346],
        [1.9228602319382888, 1.3578862029805236],
        [2.084538817942865, 1.306790834319281],
        [2.24386397872698, 1.2684713801302274],
        [2.3993471367931374, 1.2464543256802854],
        [2.541865662743322, 1.208895335382942],
        [2.6761493830464027, 1.1567028315171182],
        [2.793379925635009, 1.110171699783217],
        [2.90040825352477, 1.0741811823138379],
        [3.009102465376286, 1.031720186727621],
        [3.135152511504502, 0.9898921502998155],
        [3.280454061444484, 0.9418997297008239],
        [3.4489415320186807, 0.887190221011452],
        [3.623645484448154, 0.8295610577658598],
        [3.7853377519541564, 0.7918150023869245],
        [3.9253994387279287, 0.7595711847002404],
        [4.046646373277971, 0.7262077511707157],
        [4.1616084962798015, 0.6980132918058594],
        [4.277620280771502, 0.6818333246803442],
        [4.389818808446994, 0.6752412724353011],
        [4.49713789457758, 0.6714839596256545],
        [4.601310171045224, 0.6714839596256545],
        [4.601310171045224, 0.6714839596256545],
        [4.702545460001113, 0.6714839596256545],
        [4.811846337183488, 0.6714839596256545],
        [4.922850240008446, 0.6714839596256545],
        [5.03324660789275, 0.6714839596256545],
        [5.13241104638218, 0.6899755162386635],
        [5.235589848843943, 0.7319439848815168],
        [5.354469048144118, 0.7752883057422463],
        [5.47773510703104, 0.8310758897335799],
        [5.62096430121747, 0.8602984546930691],
        [5.747138953195938, 0.9216860003391918],
        [5.8860982024545905, 0.9741710262138462],
        [6.007021236423924, 1.0535194411870772],
        [6.136825629366831, 1.1273794719295325],
        [6.263892539702526, 1.2073568127227046],
        [6.394702864512799, 1.286408285081309],
        [6.533446911822778, 1.3611742410284562],
        [6.633742495590361, 1.472003295376117],
        [6.7169344154420205, 1.5970826021152258],
        [6.812563221606743, 1.7117928118022976],
        [6.901990449574399, 1.8314884702910235],
        [6.989947214057651, 1.9523429595871151],
        [7.063466415904271, 2.083317769309836],
        [7.149438313220993, 2.2055233648535144],
        [7.220608368341182, 2.3375515225816614],
        [7.294470476979129, 2.4672827150239995],
        [7.369973727638089, 2.5957986539914115],
        [7.438156846545936, 2.7284234740999938],
        [7.511249392154361, 2.8580348572971235],
        [7.581101510326764, 2.9893645841901377],
        [7.66059137177806, 3.1158453965619892],
        [7.734391227723906, 3.245595045332885],
        [7.799925748914239, 3.3794866699449413],
        [7.870753925269665, 3.51128521552003],
        [7.925929656014897, 3.6500352562450677],
        [7.964414099033614, 3.7948260772226448],
        [7.9901500596334385, 3.9429532215654257],
        [8.011152060330566, 4.09072556246977],
        [8.040193581630449, 4.2340365758454],
        [8.06467785560409, 4.37709071186341],
        [8.10176082318459, 4.516203648580119],
        [8.113642721166535, 4.659409676600752],
        [8.113642721166535, 4.801729466181733],
        [8.076486013403597, 4.93967973039861],
        [8.031441137222947, 5.066713772633385],
        [7.990473310698333, 5.182249856784736],
        [7.943484615702064, 5.275695760990801],
        [7.904902066929564, 5.352202232690601],
        [7.871803400586217, 5.42160658851519],
        [7.826405094975207, 5.48693318215934],
        [7.75571972188516, 5.561687372579],
        [7.676870565485384, 5.645638669504286],
        [7.572600441285298, 5.704821229608268],
        [7.465475302082195, 5.793835991938756],
        [7.346779492318998, 5.872496111155431],
        [7.213963741681785, 5.9158662671611815],
        [7.074855865299543, 5.939588761247311],
        [6.931998152343459, 5.943022955418627],
        [6.788632860232849, 5.943022955418627],
        [6.6458729719654634, 5.943022955418627],
        [6.502887719762732, 5.9244450808120686],
        [6.362447565476686, 5.885014483129451],
        [6.223314397848914, 5.8523162604692285],
        [6.093609519706351, 5.811754411004733],
        [5.965108645035041, 5.772915631018951],
        [5.832944505197795, 5.7363958785563005],
        [5.697088654512691, 5.709456622010604],
        [5.569169642950019, 5.661514840330112],
        [5.449821837777615, 5.606323050314387],
        [5.34508450954909, 5.550494236445196],
        [5.251178471764485, 5.493744812720405],
        [5.168376312267545, 5.405917503679457],
        [5.074069670609112, 5.325330721459529],
        [4.976690560602954, 5.245231679907913],
        [4.870291538250283, 5.160318304991042],
        [4.754957417150421, 5.067549184918234],
        [4.609922616764756, 4.973552877822809],
        [4.4522618638773315, 4.871860659845819],
        [4.307376658461834, 4.762485112516666],
        [4.173602431762872, 4.6572089559253875],
        [4.032297354226342, 4.577492546763978],
        [3.9155528044247805, 4.525825547430436],
        [3.8495037417926667, 4.483914945653592],
        [3.806721782359913, 4.439504199470287],
        [3.753309611273469, 4.404324748933579],
        [3.6361080775557593, 4.386943363939919],
        [3.518421062783961, 4.363935823795957],
        [3.3852494144797998, 4.363233478421818],
        [3.258704656584081, 4.342031237921609],
        [3.148090333336518, 4.3401901526437365],
        [3.0427575626617536, 4.31373499414275],
        [2.9408333097768584, 4.288975251785791],
        [2.8399559156136096, 4.2799366936237435],
        [2.719766192500618, 4.260316847387274],
        [2.5480680586426856, 4.192210563893588],
        [2.339639616992712, 4.109405031812592],
        [2.145424020794566, 4.057985077384098],
        [1.9856763143933445, 4.043369263240979],
        [1.8383089111805362, 4.005689753962501],
        [1.706443440818894, 3.954486225833204],
        [1.5839823174849097, 3.911475062411733],
        [1.46762524049177, 3.858743238056768],
        [1.3549665632986854, 3.8236531718422113],
        [1.2455606516996427, 3.8141363522856677],
        [1.1614615972204392, 3.7708206733278877],
        [1.072761063763191, 3.7549771106705925],
        [1.0044352106024346, 3.7080044702275257],
        [0.9340178524101044, 3.666295376905524],
        [0.8731503627097648, 3.604781062574472],
        [0.822046607510027, 3.5282307071932855],
        [0.7635021414814505, 3.444225663984532],
        [0.6744289168550944, 3.35273001871707],
        [0.6020779786055518, 3.2415062534687817],
        [0.47645987960365804, 3.1213895193553545],
        [0.375705081817138, 2.982235006003526],
        [0.30362426266583675, 2.8302470990656206],
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
