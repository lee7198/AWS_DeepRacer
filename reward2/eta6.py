def reward_function(params):
    reward = 0
    
    all_wheels_on_track = params['all_wheels_on_track']
    is_left_of_center = params['is_left_of_center']
    
    if is_left_of_center == True:
        reward = 100
    else: 
        reward = 1
    
    
    if all_wheels_on_track == False:
        reward = 1e-3
    
    return float(reward)