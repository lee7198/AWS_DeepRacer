# 주행 트렉에서 직선 포인트들
straight_points = list(range(12, 38)) + list(range(48, 74)) + list(range(102, 142))


def reward_function(params):
    reward = 0

    all_wheels_on_track = params["all_wheels_on_track"]
    is_left_of_center = params["is_left_of_center"]
    abs_steering = abs(params["steering_angle"])
    waypoints = params["waypoints"]
    closest_waypoints = params["closest_waypoints"]
    next_point = waypoints[closest_waypoints[1]]

    if all_wheels_on_track == False:
        reward = 1e-3

    # 직선코스에서 조향각도 제한
    if next_point[0] in straight_points:
        if abs_steering < 7:
            reward = 15
        else:
            reward = 1
    else:
        if is_left_of_center == True:
            reward = 10
        else:
            reward = 1

    return float(reward)
