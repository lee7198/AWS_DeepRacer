# 주행 트렉에서 직선 포인트들
straight_points = list(range(12, 38)) + list(range(102, 142))

print(len(straight_points))


def reward_function(params):
    reward = 0

    all_wheels_on_track = params["all_wheels_on_track"]
    is_left_of_center = params["is_left_of_center"]
    abs_steering = abs(params["steering_angle"])
    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    track_width = params["track_width"]
    distance_from_center = params["distance_from_center"]

    next_point = waypoints[closest_waypoints[1]]
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # 직선코스에서 조향각도 제한
    if next_point[0] in straight_points:
        if abs_steering < 5:
            reward = 15
        else:
            reward = 1
    else:
        if is_left_of_center == True:
            if distance_from_center <= marker_1:
                reward = 5
            elif distance_from_center <= marker_2:
                reward = 10
            elif distance_from_center <= marker_3:
                reward = 15
        else:
            reward = 1

    if all_wheels_on_track == False:
        reward = 1e-3

    return float(reward)
