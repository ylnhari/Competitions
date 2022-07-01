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
class OLD:
    speed = None
    steering_angle = None 
    steps = None
    direction_diff = None
    distance_from_center = None
    progress = None

def get_slope(next_point, prev_point):
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)
    return  track_direction

def reward_function(params):
    
    # Read input parameters
    heading = params['heading']
    distance_from_center = params['distance_from_center']
    steps = params['steps']
    steering_angle = params['steering_angle']
    speed = params['speed']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    is_reversed  = params['is_reversed']
    is_crashed = params['is_crashed']
    track_width = params['track_width']
    # GLOBAL VARIABLES
    DIRECTION_THRESHOLD = 10
    MAX_SPEED = 4
    MIN_SPEED = 1

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(get_slope(next_point, prev_point) - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if direction_diff < DIRECTION_THRESHOLD:
        is_heading_in_right_direction = True
    else:
        is_heading_in_right_direction = False
    
    is_turn_upcoming = False
    closest_next_point = closest_waypoints[1]
    if closest_next_point + 2 <= len(waypoints) - 1:
        next_points = [closest_next_point, closest_next_point + 1, closest_next_point + 2]
        slopes = []
        for i in range(0,len(next_points) - 1):
            slopes.append(get_slope(waypoints[next_points[i+1]], waypoints[next_points[i]]))
        
        if len(slopes) > 1:
            if abs(slopes[0] - slopes[1]) > 1:
                is_turn_upcoming = True


    # Reinitialize previous parameters if it is a new episode
    if OLD.steps is None or steps < OLD.steps:
        OLD.speed = None
        OLD.steering_angle = None
        OLD.direction_diff = None
        OLD.distance_from_center = None
        OLD.progress = None
    #Check if the speed has dropped
    has_speed_dropped = False
    if OLD.speed is not None:
        if OLD.speed > speed:
            has_speed_dropped = True
    speed_maintain_bonus = 1
    #Penalize slowing down without good reason on straight portions
    if has_speed_dropped and not is_turn_upcoming: 
        speed_maintain_bonus = min( speed / OLD.speed, 1 )
    #Penalize making the heading direction worse
    heading_decrease_bonus = 0
    if OLD.direction_diff is not None:
        if is_heading_in_right_direction:
            if abs( OLD.direction_diff / direction_diff ) > 1:
                heading_decrease_bonus = min(10, abs( OLD.direction_diff / direction_diff ))
    #has the steering angle changed
    has_steering_angle_changed = False
    if OLD.steering_angle is not None:
        if not(math.isclose(OLD.steering_angle,steering_angle)):
            has_steering_angle_changed = True
    steering_angle_maintain_bonus = 1 
    #Not changing the steering angle is a good thing if heading in the right direction
    if is_heading_in_right_direction and not has_steering_angle_changed:
        if abs(direction_diff) < 10:
            steering_angle_maintain_bonus *= 2
        if abs(direction_diff) < 5:
            steering_angle_maintain_bonus *= 2
        if OLD.direction_diff is not None and abs(OLD.direction_diff) > abs(direction_diff):
            steering_angle_maintain_bonus *= 2
    #Reward reducing distance to the race line
    distance_reduction_bonus = 1
    if OLD.distance_from_center is not None and OLD.distance_from_center > distance_from_center:
        if abs(distance_from_center) > 0:
            distance_reduction_bonus = min( abs( OLD.distance_from_center / distance_from_center ), 2)

    heading = params['heading']
    vehicle_x = params['x']
    vehicle_y = params['y']
    current_position = [vehicle_x, vehicle_y]
    if closest_waypoints[1] < len(waypoints) - 1:
        next_route_point = waypoints[closest_waypoints[1] + 1]
    else:
        next_route_point = waypoints[closest_waypoints[1]]

    route_direction = get_slope(next_route_point, current_position)
    
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = route_direction - heading
    #Check that the direction_diff is in valid range
    #Then compute the heading reward
    heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 10
    if abs(direction_diff) <= 20:
        heading_reward = math.cos( abs(direction_diff ) * ( math.pi / 180 ) ) ** 4

    # for every 10 percent 
    progress_mark = progress % 10

    reward_for_every_10_percent_progress = 0
    if OLD.progress is not None and OLD.progress > progress_mark:
        reward_for_every_10_percent_progress =  progress_mark/10

    # speed reward
    speed_reward = get_exponential(MAX_SPEED - speed, MAX_SPEED - MIN_SPEED)


    #distance reward is value of the standard normal scaled back to 1. #Hence the 1/2*pi*sigma term is cancelled out

    distance_reward = get_exponential(distance_from_center, track_width*0.5)
    #heading component of reward
    HC = ( 1 * heading_reward * steering_angle_maintain_bonus * heading_decrease_bonus)
    #distance component of reward
    DC = ( 1 * distance_reward * distance_reduction_bonus * reward_for_every_10_percent_progress)
    #speed component of reward
    SC = ( 0.5 * speed_reward * speed_maintain_bonus )
    #Immediate component of reward
    IC = ( HC + DC + SC ) ** 2 + ( HC * DC * SC )
    
    if is_reversed: #  penalize if car is reversed
        IC =   0.0003
    if is_crashed: # penalize if car is crashed
         IC =  0.0003
    if not all_wheels_on_track: # penalize if wheels go off track
         IC = 0.0003

    
    # Before returning reward, update the variables
    OLD.speed = speed
    OLD.steering_angle = steering_angle
    OLD.direction_diff = direction_diff
    OLD.steps = steps
    OLD.distance_from_center = distance_from_center
    OLD.progress = progress

    return max(IC ,1e-3)