# AWS DR Reward function

- AWS DeepRacer는 강화학습으로 작동함

### Parameters of reward functions

| Params               | Description                               |
| -------------------- | ----------------------------------------- |
| x & y                | 트랙상에서 차량의 위치                    |
| heading              | 트랙상 차량의 방향                        |
| waypoints            | 웨이포인트 좌표 목록                      |
| closest_waypoints    | 차량에 가장 가까운 두 웨이포인트의 인덱스 |
| progress             | 완주한 트랙의 퍼센트                      |
| steps                | 완주한 단계 수                            |
| track_width          | 트랙의 너비                               |
| distance_from_center | 트랙 중심선으로부터의 거리                |
| is_left_of_center    | 차량이 중앙선 왼쪽에 있는지 여부          |
| all_wheels_on_track  | 차량이 완전히 트랙 경계 내에 있는지 여부  |
| speed                | 차량의 관측 속도                          |
| steering_angle       | 전륜 바퀴의 조향 각도                     |

## Reward function

이러한 모든 매개변수를 사용하여 운전 행동을 장려하는 reward function 를 정의할 수 있습니다.

보상 함수의 몇 가지 예와 매개변수를 사용하여 보상을 결정하는 방법을 살펴보겠습니다. 다음 세 가지 reward function 는 AWS DeepRacer 콘솔에서 예제로 제공되므로 이들을 시도하여 어떻게 작동하는지 또는 AWS DeepRacer 리그에 제출할 수 있습니다.

### 1. **Stay On Track exaple**

```python
def reward_function(params):
    '''
    Example of rewarding the agent to stay inside the two borders of the track
    '''

    # Read input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']

    # Give a very low reward by default
    reward = 1e-3

    # Give a high reward if no wheels go off the track and
    # the agent is somewhere in between the track borders
    if all_wheels_on_track and (0.5*track_width - distance_from_center) >= 0.05:
        reward = 1.0

    # Always return a float value
    return float(reward)
```

`all_wheels_on_track` , `distance_from_center` , `track_width` 의 params를 사용하여 자동차의 상태를 추적하고 높은 리워드를 제공합니다.

> 이 함수는 트랙을 벗어나지 않는 것 외에는 특정한 행동을 보상하지 않기 때문에, 이 함수로 훈련한 에이전트는 특정한 행동에 수렴하는 데 더 오랜 시간이 걸릴 수 있습니다.

### 2. Follow the center line example

```python
def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    '''

    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']

    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.5
    elif distance_from_center <= marker_3:
        reward = 0.1
		else:
        reward = 1e-3  # likely crashed/ close to off track

    return float(reward)
```

`distance_from_center` , `track_width` 를 사용하여 차량이 트렉 중심에서 멀어질 수록 보상이 감소하는 방식으로 보상을 반환합니다.

이 예제는 보상할 주행 행동의 종류에 대해 더 구체적으로 설명하므로,이 함수로 훈련 된 에이전트는 트랙을 따르는 것을 잘 배우지만 코너에서 가속하거나 브레이크하는 것과 같은 다른 행동을 배우기는 어렵습니다.

### 3. Prevent zig-zag

```python
def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    abs_steering = abs(params['steering_angle']) # Only need the absolute steering angle

		# Calculate 3 marks that are farther and father away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

		# Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.5
    elif distance_from_center <= marker_3:
        reward = 0.1
    else:
        reward = 1e-3  # likely crashed/ close to off track

    # Steering penality threshold, change the number based on your action space setting
		ABS_STEERING_THRESHOLD = 15

    # Penalize reward if the car is steering too much
    if abs_steering > ABS_STEERING_THRESHOLD:
        reward *= 0.8

    return float(reward)
```

이 예는 에이전트가 중앙선을 따르도록 인센티브를 제공하지만 너무 많이 조향할 경우 낮은 보상으로 불이익을 주므로 지그재그 행동을 방지하는 데 도움이 됩니다.

에이전트는 시뮬레이터에서 원활하게 운전하는 방법을 학습하여 실제 차량에 배치되었을 때 동일한 행동을 보일 가능성이 높습니다.
