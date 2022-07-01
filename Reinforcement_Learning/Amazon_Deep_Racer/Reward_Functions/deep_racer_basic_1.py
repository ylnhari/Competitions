import math

def get_exponential(x, highest_x_possible):
    try:
        if highest_x_possible > 3:
            multiple = 3
        else:
            multiple = 1
        divisor = highest_x_possible / multiple
        x1 = x/divisor
        highest_x_possible1 = highest_x_possible/divisor
        exp_0 = math.exp(0)
        num = (math.exp(x1) - exp_0)
        den = (math.exp(highest_x_possible1) - exp_0)
        return 1 - (num/den)
    except ZeroDivisionError:
        return 1

def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    abs_steering = abs(params['steering_angle']) # Only need the absolute steering angle
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    is_reversed  = params['is_reversed']
    is_crashed = params['is_crashed']
    steps = params['steps']
    progress = params['progress']
    speed = abs(params['speed'])

    # GLOBAL Constants
    SPEED_THRESHOLD = 2
    MAX_SPEED = 4
    MIN_SPEED = 1
    # Steering penality threshold, change the number based on your action space setting
    ABS_STEERING_THRESHOLD = 20
    MAX_ABS_STEERING_THRESHOLD = 30

    if is_reversed: #  penalize if car is reversed
        return  0.0003

    if is_crashed: # penalize if car is crashed
        return 0.0003

    if not all_wheels_on_track: # penalize if wheels go off track
        return 0.0003
    else:
        reward_for_staying_in_lane = get_exponential(distance_from_center, track_width*0.5)

    # Penalize if the carbased on speed (Higher the Speed , Higher the reward,)
    reward_for_speed =  get_exponential(MAX_SPEED - speed, MAX_SPEED - MIN_SPEED)

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large  (small difference -> high reward)
    if direction_diff > 90:
        reward_based_on_waypoint_heading = get_exponential(direction_diff, 90)
    else:
        reward_based_on_waypoint_heading = 2 * get_exponential(direction_diff, 90)

    # reward for steering angle aligning with track direction
    #reward_for_steering_aligned_to_track = 1 - (abs(params['steering_angle'] - direction_diff) / 180.0)

    reward_for_steering = get_exponential(abs_steering, MAX_ABS_STEERING_THRESHOLD)
    # Penalize reward if the car is steering too much
    if abs_steering > ABS_STEERING_THRESHOLD:
        reward_for_steering *= 0.8

    # reward_for_speed_steering = ((2 * speed * speed) / abs_steering + 1)

    reward = reward_for_staying_in_lane + reward_for_speed + reward_based_on_waypoint_heading

    return float(reward)