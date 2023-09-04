# Racing line using Waypoints


def reward_function(params):
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
        85,
        86,
        87,
        88,
        89,
        90,
        91,
        92,
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
        139,
        140,
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
        135,
        136,
        137,
        138,
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
        132,
        133,
        134,
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
        152,
    ]
    slow = [84, 85, 139, 140]

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
