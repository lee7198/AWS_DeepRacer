import math

left = [
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    34,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    47,
    48,
    49,
    50,
    51,
    52,
    53,
    54,
    55,
    56,
    57,
    58,
    59,
    60,
    61,
    62,
    63,
    64,
    65,
    66,
    67,
    68,
    69,
    85,
    86,
    87,
    88,
    89,
    90,
    91,
    92,
    139,
    140,
    141,
    142,
    143,
    144,
    145,
    146,
    147,
    148,
    149,
    150,
    151,
]
centerleft = [
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    16,
    17,
    18,
    31,
    32,
    33,
    70,
    71,
    72,
    73,
    74,
    75,
    76,
    77,
    78,
    79,
    80,
    81,
    82,
    83,
    84,
    93,
    94,
    95,
    96,
    97,
    98,
    99,
    100,
    101,
    102,
    103,
    104,
    105,
    106,
    107,
    108,
    109,
    110,
    111,
    112,
    113,
    114,
    115,
    116,
    137,
    138,
    152,
    153,
    154,
]
centerright = [
    19,
    20,
    21,
    22,
    23,
    26,
    27,
    28,
    29,
    30,
    117,
    118,
    119,
    133,
    134,
    135,
    136,
]
right = [24, 25, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132]
fast = [
    2,
    3,
    4,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
    33,
    47,
    48,
    49,
    50,
    51,
    52,
    53,
    54,
    55,
    56,
    57,
    58,
    59,
    60,
    61,
    62,
    63,
    64,
    65,
    66,
    67,
    68,
    69,
    70,
    71,
    72,
    73,
    74,
    75,
    76,
    77,
    78,
    79,
    80,
    96,
    97,
    98,
    99,
    100,
    101,
    102,
    103,
    104,
    105,
    106,
    107,
    110,
    111,
    112,
    113,
    114,
    115,
    116,
    117,
    118,
    123,
    124,
    125,
    126,
    127,
    128,
    129,
    130,
    131,
    132,
    133,
    134,
    135,
    136,
    137,
    138,
    152,
    153,
    154,
]
medium = [
    1,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    34,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42,
    43,
    44,
    45,
    46,
    81,
    82,
    83,
    86,
    87,
    88,
    89,
    90,
    91,
    92,
    93,
    94,
    95,
    108,
    109,
    119,
    120,
    121,
    122,
    141,
    142,
    143,
    144,
    145,
    146,
    147,
    148,
    149,
    150,
    151,
]
slow = [84, 85, 139, 140]


def dist(point1, point2):
    # 두 점 사이의 거리를 계산하는 함수
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5


def rect(r, theta):
    """
    극 좌표계를 직교 좌표계로 변환하는 함수

    :param r: 반지름
    :param theta: 각도 (도 단위)
    :return: (x, y) 좌표값을 담은 튜플 반환
    """

    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y


def polar(x, y):
    """
    직교 좌표계를 극 좌표계로 변환하는 함수

    :param x: x 좌표값
    :param y: y 좌표값
    :return: 극 좌표계로 변환된 (r, theta) 값 반환 (각도는 도 단위)
    """

    r = (x**2 + y**2) ** 0.5
    theta = math.degrees(math.atan2(y, x))
    return r, theta


def angle_mod_360(angle):
    """
    각도를 -180부터 +180 범위로 매핑합니다.

    예시:
    angle_mod_360(362) == 2
    angle_mod_360(270) == -90

    :param angle: 각도 (도 단위)
    :return: -180부터 +180 범위의 각도 반환
    """

    n = math.floor(angle / 360.0)

    angle_between_0_and_360 = angle - n * 360.0

    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360


def get_waypoints_ordered_in_driving_direction(params):
    # waypoints는 항상 시계 반대 방향으로 제공됩니다.
    if params["is_reversed"]:
        # 시계 방향으로 주행 중인 경우 웨이 포인트 순서를 뒤집습니다.
        return list(reversed(params["waypoints"]))
    else:
        # 시계 반대 방향으로 주행 중인 경우 웨이 포인트 순서 그대로 사용합니다.
        return params["waypoints"]


def up_sample(waypoints, factor):
    """
    제공된 웨이 포인트 사이에 추가적인 웨이 포인트를 추가합니다.

    :param waypoints: 웨이 포인트 리스트
    :param factor: 정수 값입니다. 예를 들어 결과 리스트에는 입력보다 세 배 많은 포인트가 있습니다.
    :return: 새로운 웨이 포인트 리스트 반환
    """

    p = waypoints
    n = len(p)

    return [
        [
            i / factor * p[(j + 1) % n][0] + (1 - i / factor) * p[j][0],
            i / factor * p[(j + 1) % n][1] + (1 - i / factor) * p[j][1],
        ]
        for j in range(n)
        for i in range(factor)
    ]


def get_target_point(params):
    # 주행 방향에 따라 정렬된 웨이포인트를 얻고, 그 사이에 추가적으로 웨이포인트를 생성합니다.
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)

    # 자동차의 현재 위치
    car = [params["x"], params["y"]]

    # 각 웨이포인트와 자동차 사이의 거리를 계산합니다.
    distances = [dist(p, car) for p in waypoints]

    # 가장 가까운 거리와 그 인덱스를 찾습니다.
    min_dist = min(distances)
    i_closest = distances.index(min_dist)

    n = len(waypoints)

    # 가장 가까운 점에서 시작하여 순서대로 정렬된 새로운 경로 점 배열을 생성합니다.
    waypoints_starting_with_closest = [waypoints[(i + i_closest) % n] for i in range(n)]

    r = params["track_width"] * 0.9

    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]

    # 첫 번째 외부 지점을 찾습니다.
    i_first_outside = is_inside.index(False)

    if i_first_outside < 0:
        # 이 경우는 전체 트랙 크기만큼 r을 선택한 경우에만 발생할 수 있습니다. 이때는 가장 가까운 점을 반환합니다.
        return waypoints[i_closest]

    return waypoints_starting_with_closest[i_first_outside]


def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params["x"]
    car_y = params["y"]

    dx = tx - car_x
    dy = ty - car_y

    heading = params["heading"]

    _, target_angle = polar(dx, dy)

    steering_angle = target_angle - heading

    return angle_mod_360(steering_angle)


def score_steer_to_point_ahead(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params["steering_angle"]

    error = (steering_angle - best_stearing_angle) / 60.0  # 60도는 이미 매우 나쁜 상황을 의미합니다.

    score = 1.0 - abs(error)

    return max(score, 0.01)  # 최적화기는 음수와 너무


def assign_waypoints(params):
    closest = params["closest_waypoints"]
    nextwaypoint = max(closest[0], closest[1])
    if params["all_wheels_on_track"] == True:
        if nextwaypoint in centerleft:
            if (params["distance_from_center"] / params["track_width"]) <= 0.25 and (
                params["is_left_of_center"]
            ):
                reward = 14
            elif (params["distance_from_center"] / params["track_width"]) <= 0.25 and (
                not params["is_left_of_center"]
            ):
                reward = 0
            else:
                reward = -7

        elif nextwaypoint in centerright:
            if (params["distance_from_center"] / params["track_width"]) <= 0.25 and (
                not params["is_left_of_center"]
            ):
                reward = 14
            elif (params["distance_from_center"] / params["track_width"]) <= 0.25 and (
                params["is_left_of_center"]
            ):
                reward = 0
            else:
                reward = -7

        elif nextwaypoint in left:
            if (
                (params["is_left_of_center"])
                and (params["distance_from_center"] / params["track_width"]) > 0.25
                and (params["distance_from_center"] / params["track_width"]) < 0.48
            ):
                reward = 14
            else:
                reward = -7
        elif nextwaypoint in right:
            if (
                (not params["is_left_of_center"])
                and (params["distance_from_center"] / params["track_width"]) > 0.25
                and (params["distance_from_center"] / params["track_width"]) < 0.48
            ):
                reward = 14
            else:
                reward = -7

        if nextwaypoint in fast:
            if params["speed"] == 3:
                reward += 14
            else:
                reward -= (5 - params["speed"]) ** 2
        elif nextwaypoint in medium:
            if params["speed"] == 2:
                reward += 14
            else:
                reward -= 7
        elif nextwaypoint in slow:
            if params["speed"] == 1:
                reward += 14
            else:
                reward -= (2 + params["speed"]) ** 2
    else:
        reward = 0.001

    return float(reward)


def reward_function(params):
    return float(score_steer_to_point_ahead(params) + assign_waypoints(params))
