from math import sqrt

class Motion():
    def calculate_speed_and_distance(init_speed, acceleration, unit_time):
        """ calculates the speed and distance at the next time step based on the initial speed, acceleration given, and unit time given.
        Args:
            init_speed (float): Initial vehicle speed.
            acceleration (float): vehicle acceleration level to the next time step.
            unit_time (float): analysis unit time to deci-second.
        Returns:
            tuple: the new speed and covered distance.
        """
        new_speed = init_speed + acceleration * unit_time
        if new_speed >= 0: traveled_distance = init_speed*unit_time + 0.5 * acceleration * unit_time**2
        else:
            time_to_stop = max(0, -init_speed/acceleration)
            new_speed = 0
            traveled_distance = init_speed*time_to_stop+0.5*acceleration*time_to_stop**2

            if time_to_stop == 0: time_to_stop = unit_time
            return (new_speed, traveled_distance, time_to_stop)   
        return (new_speed, traveled_distance, unit_time)


    def calculate_speed_and_time(v_0, acceleration, distance):     
        """ calculates the new speed and travel time given the initial speed, acceleration and the traveled distance.
        Args:
            v_0 (float): Initial vehicle speed.
            acceleration (float): vehicle acceleration level.
            distance (float): traveled distance.
        Returns:
            tuple: the new speed and travel time.
        """
        v_1_squared = v_0**2 + 2 * acceleration * distance
        if v_1_squared > 0.0001:
            v_1 = sqrt(v_1_squared)
            new_time = round(2 * distance / (v_0 + v_1),3)
            v_1 = round(v_1, 3)
        elif v_1_squared < 0:
            v_1 = 0
            acceleration = (v_1**2 - v_0**2) / (2*distance)
            new_time = (v_1 - v_0) / acceleration
        else: return(-1, 0)
        return (v_1, new_time)
    
    def calculate_distance_and_time(v_0, v, a, dt=0.2):
        if v != v_0:
            t = (v - v_0)/a
            distance = (v**2 - v_0**2) / (2*a)
        else:
            distance = v_0 * dt
        return distance, dt


    def decelerate_to_stop(DTI, current_state, unit_time):
        """apply deceleration to the current state to reach the stop bar according to the emergency deceleration rule.

        Args:
            DTI (float): Distance to intersection stop bar.
            current_state (tuple): Current state variable (speed, distance, time, throttle).
            unit_time (float): analysis unit time.

        Returns:
            tuple: the next decelerated state.
        """

        try: decel_level = min(0, (0 - current_state[0]**2)/(2*DTI))
        except: decel_level = -6

        if decel_level != 0: time = -current_state[0] / decel_level
        else: time = 0.1

        if time >= unit_time and decel_level != 0:
            next_speed = current_state[0] + decel_level * unit_time
            next_dist = current_state[1] + (current_state[0] * unit_time + (0.5*decel_level*unit_time**2))
            next_time = current_state[2] + unit_time
            return (next_speed, next_dist, next_time, 0)
        elif time < unit_time and decel_level != 0:
            next_speed = 0
            next_dist = current_state[1] + DTI
            next_time = current_state[2] + time
            return (next_speed, next_dist, next_time, 0)
        else:
            next_speed = current_state[0]
            next_dist = current_state[1] + current_state[0] * unit_time
            next_time = current_state[2] + unit_time
            return (next_speed, next_dist, next_time, 0)


    def min_stop_distance(v, ac=-6.0):
        """calculate the minimum stop distance for a given speed given the emergency brake distance.

        Args:
            v (float): current/initial speed of the vehicle

        Returns:
            float: minimum distance to stop.
        """
        distance = -v**2 / (2*ac)
        return max(0, distance)