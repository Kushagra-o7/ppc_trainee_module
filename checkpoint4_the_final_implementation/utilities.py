import numpy as np

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def stanley_steering(x, y, yaw, v, waypoints, k=2.0, ks=0.1, max_steer=np.radians(30)):
    """
    Improved Stanley steering controller.
    
    Args:
        x, y     : rear axle position of the car
        yaw      : vehicle heading angle (in radians)
        v        : vehicle speed
        waypoints: Nx2 array of path waypoints
        k        : cross-track gain
        ks       : softening term to prevent division by zero
        max_steer: steering angle limits (in radians)

    Returns:
        steer       : steering angle in radians
        target_idx  : index of the nearest waypoint
    """
    # Step 1: Find nearest waypoint to rear axle
    dists = np.linalg.norm(waypoints - np.array([x, y]), axis=1)
    target_idx = np.argmin(dists)

    # Step 2: Compute heading of path at the target point
    if 0 < target_idx < len(waypoints) - 1:
        dx, dy = waypoints[target_idx + 1] - waypoints[target_idx - 1]
    elif target_idx == 0:
        dx, dy = waypoints[1] - waypoints[0]
    else:
        dx, dy = waypoints[-1] - waypoints[-2]
    path_heading = np.arctan2(dy, dx)

    # Step 3: Compute heading error
    yaw_error = normalize_angle(path_heading - yaw)

    # Step 4: Compute cross-track error
    map_x, map_y = waypoints[target_idx]
    dx = map_x - x
    dy = map_y - y

    # Perpendicular projection to determine side of the path
    heading_vector = [np.cos(yaw), np.sin(yaw)]
    error_vector = [dx, dy]
    cross = np.cross(heading_vector, error_vector)
    e = np.linalg.norm([dx, dy])
    if cross < 0:
        e *= -1  # Negative if on the right side

    # Step 5: Stanley control law
    steer = yaw_error + np.arctan2(k * e, v + ks)
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