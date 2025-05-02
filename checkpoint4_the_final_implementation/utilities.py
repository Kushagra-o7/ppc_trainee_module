import numpy as np

def stanley_steering(x, y, yaw, v, waypoints, k=1.0, ks=1e-2, max_steer=np.radians(30)):
    """
    Stanley steering controller.

    Args:
        x, y     : rear axle position of the car
        yaw      : vehicle heading angle (in radians)
        v        : vehicle speed
        waypoints: Nx2 array of path waypoints
        k        : cross-track gain
        ks       : softening term to prevent div by zero
        max_steer: steering angle limits (in radians)

    Returns:
        steer       : steering angle in radians
        target_idx  : index of the nearest waypoint
    """
    # Step 1: Compute front axle position
    L = 2.5  # assume fixed wheelbase
    fx = x + L * np.cos(yaw)
    fy = y + L * np.sin(yaw)

    # Step 2: Find nearest waypoint
    
    dists = np.linalg.norm(waypoints - np.array([fx, fy]), axis=1)
    target_idx = np.argmin(dists)

    # Step 3: Compute heading of path at that point
    if 0<target_idx<len(waypoints):
        points_diff = (waypoints[target_idx-1]-waypoints[target_idx+1])
    elif target_idx==0:
        points_diff = (waypoints[0]-waypoints[1])
    else:
        points_diff = (waypoints[target_idx-1]-waypoints[target_idx])
    
    heading= np.arctan2(points_diff[1], points_diff[0])
    
    # Step 4: Compute heading error
    yaw_error= heading - yaw

    # Step 5: Compute signed cross-track error
    sign_check= yaw < np.arctan2((waypoints[target_idx][1]-y),(waypoints[target_idx][0]-x))
    e= dists[target_idx] if sign_check else -dists[target_idx]

    # Step 6: Compute steering using Stanley law
    steer = yaw_error + np.arctan(k*e/(v+ks))
    steer = np.clip(steer, -max_steer, max_steer)

    return steer, target_idx

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error=10.0  #THIS IS THE TARGET HEIGHT  
                              #(i had put this to zero initially, doesnt really matter cuz its just the first iteration)

        self.intergral=0.0
        # you might need to add more variables here... hint: for Integral controller

    def update(self, error, dt):
        '''
        write the update function on your own
        input is error and dt
        output should be the thrust that is provided to drone
        '''
        self.intergral+=error*dt
        thrust = error*self.Kp + self.Kd*(error-self.prev_error)/dt + self.intergral*self.Ki
        self.prev_error=error
        return thrust        

def pid_throttle(vehicle,target_velocity):
    Kp = 10.0    #proportional constant
    Ki = 2.0    #integral constant
    Kd = 5.0    #derivative constant
    pid = PIDController(Kp, Ki, Kd)
    

    dt = 0.05
    error = target_velocity - vehicle.v # What should be the error?

    throttle = pid.update(error, dt)  
    return throttle