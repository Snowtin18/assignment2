import robot_params
import numpy as np 
import time

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = time.time()

def at_goal(robot_state, goal_state):    
    
    #check if we have reached goal point
    d = np.linalg.norm(np.array(goal_state)-np.array(robot_state))
    
    if d <= robot_params.goal_threshold:
        return True
    else:
        return False

def gtg(robot_state, goal_state):  
    #The Go to goal controller
    
    global prev_heading_error
    global total_heading_error  
    global previous_time
    
    #Controller parameters
    # Kp = 0.0656
    # Kd = 0.001
    # K = 0.4
    Kp = 0.00656
    Kd = 0.0001
    K = 0.4
    
    #determine how far to rotate to face the goal point
    delxy=np.subtract((np.array(goal_state[0:2])),(np.array(robot_state[0:2])))
    des_orientation=np.arctan2(delxy[1],delxy[0])
    current_orientation=robot_state[2]

    e_new = np.rad2deg(des_orientation-current_orientation)

    #Remember to restrict error to (-180 ,180)
    e_new=((e_new+180)%360)-180
    # while(e_new>180):
    #     e_new-=360
    # while(e_new<=-180):
    #     e_new+=360
    
    
    current_time = time.time()
    dt = current_time-previous_time
    previous_time = current_time
    
    #PD controller for angular velocity
    W = Kp*e_new+Kd*((e_new-prev_heading_error)/dt)
    #prev_heading_error = ??
    prev_heading_error = e_new

    #find distance to goal
    d = np.linalg.norm(np.array(goal_state[0:2])-np.array(robot_state[0:2]))
    print('d:',d)
    
    #P control for linear velocity
    V = K*d
    
    #request robot to execute velocity
    return [V,W]