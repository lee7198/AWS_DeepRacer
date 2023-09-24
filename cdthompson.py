"""
AWS DeepRacer reward function
"""
import math
import time

from numpy import array
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing, LineString

# Constants
DEBUG_LOG_ENABLED = True

# Action space constants
MAX_SPEED = 9.0
MAX_STEERING_ANGLE = 40.0

# Raceline track
RACE_LINE_WAYPOINTS = [
    [0.63069109, 2.80611932],
    [0.63367125, 2.69079621],
    [0.6467188, 2.57569291],
    [0.66972231, 2.46183988],
    [0.70251506, 2.35022569],
    [0.74487589, 2.24177514],
    [0.79652923, 2.1373277],
    [0.85714459, 2.03761659],
    [0.92633571, 1.94324872],
    [1.00365975, 1.85468625],
    [1.08861721, 1.77223051],
    [1.1806537, 1.69600972],
    [1.27916562, 1.6259728],
    [1.38351227, 1.56189156],
    [1.4930358, 1.50337335],
    [1.60708637, 1.44988322],
    [1.72504192, 1.40077175],
    [1.84630443, 1.35530161],
    [1.97025603, 1.31266612],
    [2.09617545, 1.2719922],
    [2.2231517, 1.23232374],
    [2.35576976, 1.190681],
    [2.48814156, 1.14836059],
    [2.62010372, 1.1049143],
    [2.75155515, 1.06006668],
    [2.88245693, 1.01371411],
    [3.01283929, 0.96594252],
    [3.14278403, 0.91697795],
    [3.27239538, 0.86710616],
    [3.40178871, 0.81664194],
    [3.52534292, 0.76798582],
    [3.64882806, 0.72098717],
    [3.77225054, 0.67657779],
    [3.89565744, 0.63576533],
    [4.01912628, 0.59935302],
    [4.14273194, 0.56802807],
    [4.26652265, 0.54240254],
    [4.390508, 0.52303129],
    [4.5146562, 0.51041311],
    [4.63889641, 0.50498126],
    [4.76312271, 0.50708933],
    [4.88719857, 0.51699788],
    [5.01096146, 0.53486531],
    [5.13422789, 0.56074489],
    [5.25679889, 0.59458804],
    [5.3784661, 0.63625275],
    [5.49901791, 0.68551547],
    [5.61824556, 0.74208505],
    [5.73594876, 0.8056172],
    [5.85194051, 0.87572882],
    [5.96605088, 0.95201138],
    [6.0781297, 1.03404317],
    [6.18804798, 1.12140005],
    [6.29569809, 1.21366463],
    [6.40099292, 1.31043383],
    [6.50386394, 1.41132475],
    [6.60425847, 1.51597899],
    [6.70213638, 1.62406543],
    [6.79746619, 1.73528168],
    [6.89022097, 1.84935431],
    [6.980374, 1.96603801],
    [7.06789434, 2.08511385],
    [7.15274241, 2.20638682],
    [7.23486553, 2.32968268],
    [7.31419351, 2.4548443],
    [7.39063436, 2.58172754],
    [7.46407004, 2.71019664],
    [7.5343525, 2.84011927],
    [7.60129994, 2.9713612],
    [7.6646937, 3.10378064],
    [7.72427579, 3.2372224],
    [7.77974745, 3.37151184],
    [7.83076905, 3.50644891],
    [7.87696157, 3.64180249],
    [7.91791, 3.77730515],
    [7.9531689, 3.91264892],
    [7.98227014, 4.0474821],
    [8.00473301, 4.18140776],
    [8.02007634, 4.31398396],
    [8.02783246, 4.44472614],
    [8.02756242, 4.57311169],
    [8.01887186, 4.69858671],
    [8.00142678, 4.82057488],
    [7.97496831, 4.93848799],
    [7.9393258, 5.05173771],
    [7.89442733, 5.15974804],
    [7.84030706, 5.26196766],
    [7.77710908, 5.35788145],
    [7.7050874, 5.44702069],
    [7.62460227, 5.52897101],
    [7.536113, 5.60337802],
    [7.44016781, 5.66995014],
    [7.33739134, 5.72845868],
    [7.22847048, 5.77873548],
    [7.11413941, 5.82066844],
    [6.99516423, 5.85419554],
    [6.87232807, 5.8792982],
    [6.74641682, 5.89599457],
    [6.61820582, 5.90433369],
    [6.48844757, 5.90439106],
    [6.35786039, 5.89626615],
    [6.22711782, 5.88008231],
    [6.09683868, 5.85598899],
    [5.96757762, 5.82416624],
    [5.83981608, 5.78483111],
    [5.71395379, 5.73824529],
    [5.59030113, 5.68472331],
    [5.46907261, 5.62464036],
    [5.35038212, 5.55843893],
    [5.23424046, 5.48663315],
    [5.12055594, 5.40981015],
    [5.00913994, 5.32862503],
    [4.89972666, 5.24377382],
    [4.79201344, 5.15593375],
    [4.6857631, 5.06560545],
    [4.5807927, 4.97315917],
    [4.48081878, 4.88243357],
    [4.37959748, 4.79420328],
    [4.27684354, 4.70906838],
    [4.17224941, 4.62769566],
    [4.06553767, 4.55071821],
    [3.95644174, 4.47877854],
    [3.84474817, 4.41244104],
    [3.73031118, 4.35215634],
    [3.61306059, 4.29823493],
    [3.49300409, 4.25082828],
    [3.37022468, 4.20991786],
    [3.24487511, 4.17530971],
    [3.1171707, 4.14663361],
    [2.98738247, 4.12334422],
    [2.85585139, 4.10466958],
    [2.72282052, 4.09005797],
    [2.58852158, 4.07896026],
    [2.45315288, 4.07089104],
    [2.31675601, 4.06579649],
    [2.18425079, 4.05787239],
    [2.05356546, 4.04652377],
    [1.9251617, 4.03105575],
    [1.79950825, 4.01084184],
    [1.67721911, 3.98511327],
    [1.55873527, 3.9534865],
    [1.44466638, 3.91541352],
    [1.33564671, 3.8704324],
    [1.23222981, 3.81830074],
    [1.13519251, 3.7586213],
    [1.04519953, 3.69128277],
    [0.96285535, 3.61637202],
    [0.88870748, 3.5341512],
    [0.82324005, 3.44504611],
    [0.76686864, 3.34963169],
    [0.71993756, 3.24861421],
    [0.68271928, 3.1428114],
    [0.65541543, 3.03313123],
    [0.63815905, 2.92055024],
    [0.63069109, 2.80611932],
]

# TUNING: Adjust these to find tune factors affect on reward
#
# Reward weights, always 0..1.  These are relative to one another
SPEED_FACTOR_WEIGHT = 0.0
SPEED_FACTOR_EASING = "linear"
WHEEL_FACTOR_WEIGHT = 0.0
WHEEL_FACTOR_EASING = "linear"
HEADING_FACTOR_WEIGHT = 0.0
HEADING_FACTOR_EASING = "linear"
STEERING_FACTOR_WEIGHT = 0.0
STEERING_FACTOR_EASING = "linear"
PROGRESS_FACTOR_WEIGHT = 0.0
PROGRESS_FACTOR_EASING = "linear"
LANE_FACTOR_WEIGHT = 0.0
LANE_FACTOR_EASING = "linear"
RACE_LINE_FACTOR_WEIGHT = 1.0
RACE_LINE_FACTOR_EASING = "linear"

# Globals
g_last_progress_value = 0.0
g_last_progress_time = 0.0
g_last_speed_value = 0.0
g_last_steering_angle = 0.0

# ===============================================================================
#
# REWARD
#
# ===============================================================================


def reward_function(params):
    """Reward function is:

    f(s,w,h,t,p) = 1.0 * W(s,Ks) * W(w,Kw) * W(h,Kh) * W(t,Kt) * W(p,Kp) * W(l,Kl)

    s: speed factor, linear 0..1 for range of speed from 0 to MAX_SPEED
    w: wheel factor, non-linear 0..1 for wheels being off the track and
       vehicle in danger of going off the track.  We want to use the full
       width of the track for smoothing curves so we only apply wheel
       factor if the car is hanging off the track.
    h: heading factor, 0..1 for range of angle between car heading vector
       and the track direction vector.  This is the current heading
       based on the immediate direction of the car regardless of steering.
    t: steering factor, 0..1 for steering pressure if steering the wrong
       direction to correct the heading.
    p: progress factor
    l: lane factor

    W: Weighting function: (1.0 - (1.0 - f) * Kf)
    Kx: Weight of respective factor

        Example 1:
          s = 0
          Ks = 0.5
          reward = (1.0 - ((1.0 - s) * Ks)) = 1.0 - (1.0 - 0) * 0.5 = 0.5

        Example 2:
          s = 0.25
          Ks = 1.0
          reward = (1.0 - ((1.0 - s) * Ks)) = 1.0 - (1.0 - 0.25) * 1.0 = 0.25

        Example 2:
          s = 1.0
          Ks = 0.1
          reward = (1.0 - ((1.0 - s) * Ks)) = 1.0 - (1.0 - 1.0) * 1.0 = 1.0

    params:

    from https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-reward-function-input.html

      Name                  Type                    Value(s)
      ----                  ----                    --------
      track_width           float                   0..Dtrack (varies)
      distance_from_center  float                   0..~track_width/2
      speed                 float                   0.0..5.0
      steering_angle        float                   -30..30
      all_wheels_on_track   Boolean                 True|False
      heading               float                   -180..+180
      waypoints             list of [float, float]  [[xw,0,yw,0] ... [xw,Max-1, yw,Max-1]]
      closest_waypoints     [int, int]              [0..Max-2, 1..Max-1]
      steps                 int                     0..Nstep
      progress              float                   0..100
      is_left_of_center     Boolean                 True|False
      is_reversed           Boolean                 True|False
      x                     float
      y                     float

    """

    # s: Speed Factor: ideal speed is max
    speed_factor = calculate_speed_factor(params)

    # w: Wheel Factor: apply pressure when wheels are off the track
    wheel_factor = calculate_wheel_factor(params)

    # h: Heading Factor
    heading_factor = calculate_heading_factor(params)

    # t: Steering Factor
    steering_factor = calculate_steering_factor(params)

    # p: Progress Factor: TBD
    progress_factor = calculate_progress_factor(params)

    # l: Lane Factor
    lane_factor = calculate_lane_factor(params)

    # r: Race line factor (distance from)
    race_line_factor = calculate_race_line_factor(params)

    # Log for validation
    if DEBUG_LOG_ENABLED:
        print(
            "s: %0.2f, w: %0.2f, h: %0.2f, t: %0.2f, p: %0.2f, l: %0.2f r: %0.2f"
            % (
                speed_factor,
                wheel_factor,
                heading_factor,
                steering_factor,
                progress_factor,
                lane_factor,
                race_line_factor,
            )
        )

    reward = 1.0
    reward *= apply_weight(speed_factor, SPEED_FACTOR_WEIGHT, SPEED_FACTOR_EASING)
    reward *= apply_weight(wheel_factor, WHEEL_FACTOR_WEIGHT, WHEEL_FACTOR_EASING)
    reward *= apply_weight(heading_factor, HEADING_FACTOR_WEIGHT, HEADING_FACTOR_EASING)
    reward *= apply_weight(
        steering_factor, STEERING_FACTOR_WEIGHT, STEERING_FACTOR_EASING
    )
    reward *= apply_weight(progress_factor, PROGRESS_FACTOR_WEIGHT)
    reward *= apply_weight(lane_factor, LANE_FACTOR_WEIGHT, LANE_FACTOR_EASING)
    reward *= apply_weight(
        race_line_factor, RACE_LINE_FACTOR_WEIGHT, RACE_LINE_FACTOR_EASING
    )

    return float(max(reward, 1e-3))  # make sure we never return exactly zero


# ===============================================================================
#
# RACE LINE
#
# ===============================================================================
def calculate_race_line_factor(params):
    # Reward for track position
    current_position = Point(params["x"], params["y"])
    race_line = LineString(RACE_LINE_WAYPOINTS)
    distance = current_position.distance(race_line)
    # clamp reward to range (0..1) mapped to distance (track_width..0).
    # This could be negative since the car center can be off the track but
    # still not disqualified.

    factor = 1.0 - distance / params["track_width"]
    print(
        "x %0.2f y %0.2f distance %0.2f track_width %0.2f factor %0.7f"
        % (params["x"], params["y"], distance, params["track_width"], factor)
    )
    return float(max(factor, 0.0))


# ===============================================================================
#
# SPEED
#
# ===============================================================================


def penalize_downshifting(speed):
    global g_last_speed_value
    if g_last_speed_value > speed:
        speed_factor = 1e-3
    else:
        speed_factor = 1.0
    g_last_speed_value = speed
    return speed_factor


def reward_upshifting(speed):
    global g_last_speed_value
    if g_last_speed_value < speed:
        speed_factor = 1.0
    else:
        speed_factor = 0.5
    g_last_speed_value = speed
    return speed_factor


def speed_or_acceleration(speed):
    """Reward top speed AND any acceleration as well"""
    global g_last_speed_value
    if speed > g_last_speed_value:
        speed_factor = 1.0
    else:
        speed_factor = percentage_speed(speed)
    return speed_factor


def percentage_speed(speed):
    return speed / MAX_SPEED


def calculate_speed_factor(params):
    """Calculate the speed factor"""

    # make calls here not affect each other
    speed_factor = percentage_speed(params["speed"])
    return min(speed_factor, 1.0)


# ===============================================================================
#
# PROGRESS
#
# ===============================================================================


def progress_over_time(progress):
    """Calculate the progress per time.  Note that
    we rely on how the simulation code calculates
    progress which is an unknown algorithm.

    The nice thing about this algorithm is that is scales
    up rewards exponentially, as the differences in lower
    lap times are more valueable than at higher lap times.
    """
    global g_last_progress_value
    global g_last_progress_time
    current_time = time.time()
    # progress is 0..100
    if g_last_progress_value == 0:
        progress_factor = 1.0  # arbitrary but positive enough to promote going
    else:
        # time can be anything, but probably ~20s/lap, 15fps:
        #       1s/15frames = 67ms/frame = 0.067s
        #
        #   for 30s/lap: 30s*15f/s = 400 frames
        #         => expected progress of 100/400 = 0.25 per frame
        #         => 3.7
        #
        #   assuming 20s/lap: 20s*15f/s = 300 frames
        #         => expected progress of 100/300 = 0.3 progress per frame / 0.067s
        #         => 4.47
        #
        #   for 13s/lap: 13s*15f/s = 195 frames
        #         => expected progress of 100/195 = 0.51 per frame
        #         => 7.6
        #
        #   for 12s/lap: 12s*15f/s = 180 frames
        #         => expected progress of 100/180 = 0.55 per frame
        #         => 8.2
        #
        #   for 10s/lap: 10s*15f/s = 150 frames
        #         => expected progress of 100/150 = 0.67 per frame
        #         => 10
        #
        #   for 9s/lap: 9s*15f/s = 135 frames
        #         => expected progress of 100/135 = 0.74 per frame
        #         => 11.04
        #
        #   for 8s/lap: 8s*15f/s = 120 frames
        #         => expected progress of 100/120 = 0.83 per frame
        #         => 12.39
        #
        progress_factor = (progress - g_last_progress_value) / (
            current_time - g_last_progress_time
        )

    g_last_progress_value = progress
    g_last_progress_time = current_time
    return max(progress_factor, 0.0)  # make sure not going backwards


def progress_since_last(progress):
    global g_last_progress_value
    # progress is 0..100. The logic in DR environment code ensures this always
    # increases for the episode, regardless if the car is going backward.
    if g_last_progress_value > progress:
        g_last_progress_value = 0
    progress_factor = (
        progress - g_last_progress_value
    ) / 100  # divide by 100 to get percentage of track
    g_last_progress_value = progress
    return progress_factor


def calculate_progress_factor(params):
    progress_factor = 1.0
    return min(progress_factor, 1.0)


# ===============================================================================
#
# WHEELS
#
# ===============================================================================


def all_wheels_must_be_on_track(all_wheels_on_track):
    """Return low factor if car doesn't have all its wheels on the track"""
    if not all_wheels_on_track:
        wheel_factor = 1e-3  # hard code multiplier rather than making it
        # continuous since we don't know the width of
        # the car wheelbase
    else:
        wheel_factor = 1.0
    return wheel_factor


def calculate_wheel_factor(params):
    """Calculate the wheel factor"""
    wheel_factor = all_wheels_must_be_on_track(params["all_wheels_on_track"])
    return min(wheel_factor, 1.0)


# ===============================================================================
#
# HEADING
#
# ===============================================================================


def look_ahead_heading(waypoints, current_waypoint, heading):
    """Apply pressure based on upcoming track heading"""

    track_headings = []
    v_init = current_waypoint
    for i in range(3):
        v1 = waypoints[(current_waypoint + 2 * i) % len(waypoints)]
        v2 = waypoints[(current_waypoint + 2 * i + 1) % len(waypoints)]
        track_heading = angle_of_vector([v1, v2])
        track_headings.append(track_heading)
    print(track_headings)
    return 1.0


def calculate_heading_factor(params):
    """Calculate the heading factor"""
    """
  # SUPRESS: This is too experimental while we haven't finished tracks yet
  closest_waypoints = params['closest_waypoints']
  waypoints = params['waypoints']
  heading = params['heading']

  # Calculate the immediate track angle
  wp1 = waypoints[closest_waypoints[0]]
  wp2 = waypoints[closest_waypoints[1]]
  ta1 = angle_of_vector([wp1,wp2])
  print("track angle 1: %i" % ta1)

  # h: Heading Factor: apply pressure as heading is different than track angle

  # Find closest angle, accounting for possibility of wrapping
  a = abs(ta1 - heading)
  b = abs(ta1 - (heading + 360))
  heading_delta = min(a,b)
  # hard fail if going backwards
  if heading_delta > 90:
    heading_factor = 1e-3
  elif heading_delta > 45:
    heading_factor = 0.5
  else:
    heading_factor = 1.0
  """
    heading_factor = 1.0
    heading_factor = look_ahead_heading(
        params["waypoints"], params["closest_waypoints"][0], params["heading"]
    )
    return min(heading_factor, 1.0)


# ===============================================================================
#
# STEERING
#
# ===============================================================================


def penalize_steering_change(steering_angle, greater=True, less=True):
    """
    Penalize steering changes

    @greater: penalize sharper turning
    @less: penalize straightening
    """
    global g_last_steering_angle
    if abs(steering_angle) > g_last_steering_angle and greater:
        # turning sharper
        steering_penalty = 1.0
    elif abs(steering_angle) < g_last_steering_angle and less:
        # straightening
        steering_penalty = 1.0
    else:
        steering_penalty = 0.0
    g_last_steering_angle = abs(steering_angle)
    return 1.0 - steering_penalty


def percentage_steering_angle(steering_angle):
    steering_severity = abs(steering_angle) / MAX_STEERING_ANGLE
    return max(min(1.0 - steering_severity, 1.0), 0.0)


def calculate_steering_factor(params):
    """Calculate the steering factor"""
    steering_factor = percentage_steering_angle(params["steering_angle"])
    return min(steering_factor, 1.0)


# ===============================================================================
#
# LANE
#
# ===============================================================================


def percentage_distance_from_track_center(track_width, distance_from_center):
    """Return a linear percentage distance along the track width from
    the center to the outside
    """
    # make sure not negative, in case distance_from_center is over the track_width
    distance = distance_from_center / (track_width / 2.0)
    return max(min(1.0 - distance, 1.0), 0.0)


def penalize_off_track(track_width, distance_from_center):
    if distance_from_center >= (track_width / 2.0):
        penalty = 1.0
    else:
        penalty = 0.0
    return 1.0 - penalty


def calculate_lane_factor(params):
    """Calulcate the reward for the position on the track.
    Be careful to account for the wheel factor here, possibly merge
    the two later.
    """
    lane_factor = penalize_off_track(
        params["track_width"], params["distance_from_center"]
    )
    return min(lane_factor, 1.0)


# ===============================================================================
#
# HELPER METHODS
#
# ===============================================================================


def apply_weight(factor, weight, easing="linear"):
    """Apply a weight to factor, clamping both arguments at 1.0

    Factor values will be 0..1. This function will cause the range of the
    factor values to be reduced according to:

      f = 1 - weight * (1 - factor)^easing

    In simple terms, a weight of 0.5 will cause the factor to only have weighted
    values of 0.5..1.0. If we further apply an easing, the decay from 1.0 toward
    the weighted minimum will be along a curve.
    """

    f_clamp = min(factor, 1.0)
    w_clamp = min(weight, 1.0)
    if EASING_FUNCTIONS[easing]:
        ease = EASING_FUNCTIONS[easing]
    else:
        ease = EASING_FUNCTIONS["linear"]

    return 1.0 - w_clamp * ease(1.0 - f_clamp)


def vector_of_angle(angle):
    """Unit vector of an angle in degrees."""
    return [[0.0, 0.0], [math.sin(math.radians(angle)), math.cos(math.radians(angle))]]


def angle_of_vector(vector):
    """Calculate the angle of the vector in degrees relative to
    a normal 2d coordinate system.  This is useful for finding the
    angle between two waypoints.

      vector: [[x0,y0],[x1,y1]]

    """
    rad = math.atan2(vector[1][1] - vector[0][1], vector[1][0] - vector[0][0])
    return math.degrees(rad)


#
# SCALING FUNCTIONS
#


def ease_linear(x):
    return x


def ease_quadratic(x):
    return x * x


def ease_cubic(x):
    return abs(x * x * x)


def ease_quartic(x):
    return x * x * x * x


def ease_quintic(x):
    return abs(x * x * x * x * x)


def ease_septic(x):
    return abs(x * x * x * x * x * x * x)


def ease_nonic(x):
    return abs(x * x * x * x * x * x * x * x * x)


EASING_FUNCTIONS = {
    "linear": ease_linear,
    "quadratic": ease_quadratic,
    "cubic": ease_cubic,
    "quartic": ease_quartic,
    "quintic": ease_quintic,
    "septic": ease_septic,
    "nonic": ease_nonic,
}


OVAL_TRACK_RACE_LINE = array(
    [
        [0.63069109, 2.80611932],
        [0.63367125, 2.69079621],
        [0.6467188, 2.57569291],
        [0.66972231, 2.46183988],
        [0.70251506, 2.35022569],
        [0.74487589, 2.24177514],
        [0.79652923, 2.1373277],
        [0.85714459, 2.03761659],
        [0.92633571, 1.94324872],
        [1.00365975, 1.85468625],
        [1.08861721, 1.77223051],
        [1.1806537, 1.69600972],
        [1.27916562, 1.6259728],
        [1.38351227, 1.56189156],
        [1.4930358, 1.50337335],
        [1.60708637, 1.44988322],
        [1.72504192, 1.40077175],
        [1.84630443, 1.35530161],
        [1.97025603, 1.31266612],
        [2.09617545, 1.2719922],
        [2.2231517, 1.23232374],
        [2.35576976, 1.190681],
        [2.48814156, 1.14836059],
        [2.62010372, 1.1049143],
        [2.75155515, 1.06006668],
        [2.88245693, 1.01371411],
        [3.01283929, 0.96594252],
        [3.14278403, 0.91697795],
        [3.27239538, 0.86710616],
        [3.40178871, 0.81664194],
        [3.52534292, 0.76798582],
        [3.64882806, 0.72098717],
        [3.77225054, 0.67657779],
        [3.89565744, 0.63576533],
        [4.01912628, 0.59935302],
        [4.14273194, 0.56802807],
        [4.26652265, 0.54240254],
        [4.390508, 0.52303129],
        [4.5146562, 0.51041311],
        [4.63889641, 0.50498126],
        [4.76312271, 0.50708933],
        [4.88719857, 0.51699788],
        [5.01096146, 0.53486531],
        [5.13422789, 0.56074489],
        [5.25679889, 0.59458804],
        [5.3784661, 0.63625275],
        [5.49901791, 0.68551547],
        [5.61824556, 0.74208505],
        [5.73594876, 0.8056172],
        [5.85194051, 0.87572882],
        [5.96605088, 0.95201138],
        [6.0781297, 1.03404317],
        [6.18804798, 1.12140005],
        [6.29569809, 1.21366463],
        [6.40099292, 1.31043383],
        [6.50386394, 1.41132475],
        [6.60425847, 1.51597899],
        [6.70213638, 1.62406543],
        [6.79746619, 1.73528168],
        [6.89022097, 1.84935431],
        [6.980374, 1.96603801],
        [7.06789434, 2.08511385],
        [7.15274241, 2.20638682],
        [7.23486553, 2.32968268],
        [7.31419351, 2.4548443],
        [7.39063436, 2.58172754],
        [7.46407004, 2.71019664],
        [7.5343525, 2.84011927],
        [7.60129994, 2.9713612],
        [7.6646937, 3.10378064],
        [7.72427579, 3.2372224],
        [7.77974745, 3.37151184],
        [7.83076905, 3.50644891],
        [7.87696157, 3.64180249],
        [7.91791, 3.77730515],
        [7.9531689, 3.91264892],
        [7.98227014, 4.0474821],
        [8.00473301, 4.18140776],
        [8.02007634, 4.31398396],
        [8.02783246, 4.44472614],
        [8.02756242, 4.57311169],
        [8.01887186, 4.69858671],
        [8.00142678, 4.82057488],
        [7.97496831, 4.93848799],
        [7.9393258, 5.05173771],
        [7.89442733, 5.15974804],
        [7.84030706, 5.26196766],
        [7.77710908, 5.35788145],
        [7.7050874, 5.44702069],
        [7.62460227, 5.52897101],
        [7.536113, 5.60337802],
        [7.44016781, 5.66995014],
        [7.33739134, 5.72845868],
        [7.22847048, 5.77873548],
        [7.11413941, 5.82066844],
        [6.99516423, 5.85419554],
        [6.87232807, 5.8792982],
        [6.74641682, 5.89599457],
        [6.61820582, 5.90433369],
        [6.48844757, 5.90439106],
        [6.35786039, 5.89626615],
        [6.22711782, 5.88008231],
        [6.09683868, 5.85598899],
        [5.96757762, 5.82416624],
        [5.83981608, 5.78483111],
        [5.71395379, 5.73824529],
        [5.59030113, 5.68472331],
        [5.46907261, 5.62464036],
        [5.35038212, 5.55843893],
        [5.23424046, 5.48663315],
        [5.12055594, 5.40981015],
        [5.00913994, 5.32862503],
        [4.89972666, 5.24377382],
        [4.79201344, 5.15593375],
        [4.6857631, 5.06560545],
        [4.5807927, 4.97315917],
        [4.48081878, 4.88243357],
        [4.37959748, 4.79420328],
        [4.27684354, 4.70906838],
        [4.17224941, 4.62769566],
        [4.06553767, 4.55071821],
        [3.95644174, 4.47877854],
        [3.84474817, 4.41244104],
        [3.73031118, 4.35215634],
        [3.61306059, 4.29823493],
        [3.49300409, 4.25082828],
        [3.37022468, 4.20991786],
        [3.24487511, 4.17530971],
        [3.1171707, 4.14663361],
        [2.98738247, 4.12334422],
        [2.85585139, 4.10466958],
        [2.72282052, 4.09005797],
        [2.58852158, 4.07896026],
        [2.45315288, 4.07089104],
        [2.31675601, 4.06579649],
        [2.18425079, 4.05787239],
        [2.05356546, 4.04652377],
        [1.9251617, 4.03105575],
        [1.79950825, 4.01084184],
        [1.67721911, 3.98511327],
        [1.55873527, 3.9534865],
        [1.44466638, 3.91541352],
        [1.33564671, 3.8704324],
        [1.23222981, 3.81830074],
        [1.13519251, 3.7586213],
        [1.04519953, 3.69128277],
        [0.96285535, 3.61637202],
        [0.88870748, 3.5341512],
        [0.82324005, 3.44504611],
        [0.76686864, 3.34963169],
        [0.71993756, 3.24861421],
        [0.68271928, 3.1428114],
        [0.65541543, 3.03313123],
        [0.63815905, 2.92055024],
        [0.63069109, 2.80611932],
    ]
)
