import math
import numpy as np
try: 
    from src.core.models.dynamics import Dynamics
    from src.core.models.va_model import VanAerdeModel
except: 
    from dynamics import Dynamics
    from va_model import VanAerdeModel


# from va_model import VanAerdeModel
# from dynamics import Dynamics


class FRModel:
    a, b, d = None, None, None
    # a, b, d = 1.047265625, 19.046875, 0.1
    # a, b, d = 0.1, 10, 0.1  # automated driving parameters
    

    def calc_tau(current_u, u_f, q_c, k_j, u_c):
        """calculate perception-reaction time tau.

        Args:
            current_u (float): current vehicle speed u_n+1 (m/s).
            u_f (float): free flow speed (m/s).
            q_c (float): capacity flow rate (veh/s).
            k_j (float): Jam density (veh/m).
            u_c (float): Speed at capacity (m/s).

        Returns:
            float: tau (s).
        """
        current_u = min(current_u, 0.99*u_f)
        try: result = (1/k_j/u_c/u_c) * ((k_j*u_c*u_c/q_c) - u_f + ((u_f - u_c)**2 / u_f / (u_f-current_u)))
        except: result = 1.45
        return result


    def calc_desired_u(current_S, u_f, q_c, k_j, u_c):
        """calculate desired speed u for the current spacing.

        Args:
            current_S (float): current spacing (m).
            u_f (float, optional): free flow speed (m/s). Defaults to 56.1/3.6.
            q_c (float, optional): capacity flow rate (veh/s). Defaults to 1800.0/3600.
            k_j (float, optional): Jam density (veh/m). Defaults to 160.0/1000.
            u_c (float, optional): Speed at capacity (m/s). Defaults to 40.0/3.6.
        
        Returns:    
            float: desired speed u (m/s).
        """
        return round(max(0, VanAerdeModel.calc_va_u(current_S, u_f, q_c, k_j, u_c)),1)


    def calc_desired_S(u_np1, s_np1, u_n, u_f, q_c, k_j, u_c):
        """calculate desired spacing S for the current speed under the two FR Model conditions.
        
        Args:
            u_np1 (float): current vehicle speed u_n+1 (m/s).
            s_np1 (float): current vehicle spacing s_n+1 (m/s).
            u_n (float): leader vehicle speed u_n (m/s).
            u_f (float, optional): free flow speed (m/s). Defaults to 56.1/3.6.
            q_c (float, optional): capacity flow rate (veh/s). Defaults to 1800.0/3600.
            k_j (float, optional): Jam density (veh/m). Defaults to 160.0/1000.
            u_c (float, optional): Speed at capacity (m/s). Defaults to 40.0/3.6.
        
        Returns:
            float: desired spacing S (m).
        """
        jam_S = 1/k_j
        c1, c2, c3 = VanAerdeModel.calc_van_aerde_parameters(u_f, q_c, k_j, u_c)
        if u_n <= u_np1:
            tau = abs(FRModel.calc_tau(u_np1, u_f, q_c, k_j, u_c))
            result = jam_S + tau * u_np1
            # result = max(jam_S + tau * u_np1, c1 + c2/(u_f - u_np1+0.01) + c3*u_np1)
        else:
            d_desired = -3          #m/s2

            # result = max(jam_S, c1 + c2/(u_f - u_n+0.01) + c3*u_n - ((u_np1**2 - u_n**2 - math.sqrt((u_np1**2 - u_n**2)**2)) / (4*d_desired)) )
            result = max(jam_S, c1 + c2/(u_f - u_np1+0.01) + c3*u_np1 - ((u_np1**2 - u_n**2 - math.sqrt((u_np1**2 - u_n**2)**2)) / (4*d_desired)) )
            # result = max(jam_S, c1 + c2/(u_f - u_np1+0.01) + c3*u_np1)
            
        return result


    def calc_x(current_S, desired_S, current_u, desired_u):
        if desired_u == 0:
            X = 1
        else:
            if current_u < 17.8:
                X = (desired_S/current_S) * ((current_u+0.001)/(desired_u+0.001))
            else: 
                X = (max(current_S, desired_S)/current_S)
            # X = (desired_S/current_S)
            # X = ((current_u+0.001)/(desired_u+0.001))
        
        if X >= 0: pass
        else: print('Error: ', current_S, desired_S, current_u, desired_u)
        return X


    def calc_throttle(v, X):
        # a=1.047265625, b=19.046875, d= 0.1 # calibrated parameters for field test
        # a=0.3, b=10, d=0.5    # parameters used in NB work
        # a=0.25, b=10, d=0.25        # parameters used in ECO-CACC work
        # a=0.4, b=100, d=0.7d
        # a=0.1, b=100, d=0.1   # very very aggressive driving behavior
        # a=1, b=100, d=1
        # default a,b,d values are chosen to have aggressive driving behavior
        # print(x,a,b,d)
        throttle_cf = math.exp(-1*FRModel.a*X) * (1-X**(FRModel.b) * math.exp(FRModel.b*(1-X)))**FRModel.d
        # max_throttle = Dynamics.get_max_throttle(v)

        # if math.isnan(result): print(a,b,d,x, result)
        return throttle_cf
    
    def calc_throttle2(v, X, a,b,d):
        # a=1.047265625, b=19.046875, d= 0.1 # calibrated parameters for field test
        # a=0.3, b=10, d=0.5    # parameters used in NB work
        # a=0.25, b=10, d=0.25        # parameters used in ECO-CACC work
        # a=0.4, b=100, d=0.7d
        # a=0.1, b=100, d=0.1   # very very aggressive driving behavior
        # a=1, b=100, d=1
        # default a,b,d values are chosen to have aggressive driving behavior
        # print(x,a,b,d)
        throttle_cf = math.exp(-1*a*X) * (1-X**(b) * math.exp(b*(1-X)))**d
        # max_throttle = Dynamics.get_max_throttle(v)

        # if math.isnan(result): print(a,b,d,x, result)
        return throttle_cf


    def get_a_max(current_u, G):
        return Dynamics.get_max_acceleration(current_u,G)


    ## Main function
    # u_f=16.7, q_c=1800.0/3600, k_j=160.0/1000, u_c=40.0/3.6
    def get_a(prev_a, current_u, current_S, leader_u, curr_segment, consider_jerk, max_jerk):
        u_f = curr_segment.u_f
        q_c = curr_segment.q_c
        k_j = curr_segment.k_j
        u_c = curr_segment.u_c
        G = curr_segment.grade


        jam_S = 1/k_j
        # if current_S < jam_S: print("Current spacing = {0}, Minimum Spacing = {1}, Potential Collision Warnning!".format(current_S, jam_S))
        leader_u = max(0, leader_u)
        current_u = max(0, current_u)
    
        
        desired_u = FRModel.calc_desired_u(current_S, u_f, q_c, k_j, u_c)
        d_desired = -3                                      #m/s2
        desired_S = FRModel.calc_desired_S(current_u, current_S, leader_u, u_f, q_c, k_j, u_c)
        # if current_u >= u_f - 0.1: desired_S = min(desired_S, current_S)
        

        if current_S > 10000 and current_u > 0: current_S = desired_S
        elif leader_u == 0: desired_S = current_S

        X = FRModel.calc_x(current_S, desired_S, current_u, desired_u)
        


        if  X < 1: 
            f_b = FRModel.calc_throttle(current_u, X)
            a_max = Dynamics.get_max_acceleration(current_u,G)
            result =f_b*a_max
        else:
            # collision_avoidance = max(-6, (current_u**2 - leader_u**2 + math.sqrt((current_u**2 - leader_u**2)**2))**2 / (16*(d_desired + 9.81*G) * (current_S - jam_S)**2))
            # collision_avoidance = max(-9.81, (current_u**2 - leader_u**2 + math.sqrt((current_u**2 - leader_u**2)**2))**2 / (16*(d_desired + 9.81*G) * (max(current_S,jam_S) - jam_S+ 0.01)**2))
            d_required = -(current_u**2 - leader_u**2 + math.sqrt((current_u**2 - leader_u**2)**2)) / (4*(max(current_S,jam_S) - jam_S+ 0.001))
            collision_avoidance = max(-0.9*9.81, d_required**2 / (d_desired + 9.81*G))
            result = min(collision_avoidance, d_required)
            # result = collision_avoidance

        
        # print(X, f_b, a_max, f_b*a_max, "Current Spacing: ",current_S,"Desired Spacing: ",desired_S, "Current Speed: ",current_u, "Desired Speed: ", desired_u, "ACC: ", result, leader_u)
        # if result < 0: result = math.floor(1000*result)/1000


        # max_jerk = 0.5
        if prev_a >= 0 and result >= 0 and consider_jerk: result = min(result, prev_a + 0.1*max_jerk)
        return result
        # return min(FRModel.get_a_ub(current_S-jam_S, 1, current_u), result)
    
    def get_a_2(a, b, d, prev_a, current_u, current_S, leader_u, u_f, q_c, k_j, u_c, G, consider_jerk):
        jam_S = 1/k_j
        leader_u = max(0, leader_u)
        current_u = max(0, current_u)
    
    
        desired_u = FRModel.calc_desired_u(current_S, u_f, q_c, k_j, u_c)
        d_desired = -3                                       #m/s2
        desired_S = FRModel.calc_desired_S(current_u, current_S, leader_u, u_f, q_c, k_j, u_c)
        # if current_u >= u_f - 0.1: desired_S = min(desired_S, current_S)
        

        if current_S > 10000: current_S = desired_S
        elif leader_u == 0: desired_S = current_S

        X = FRModel.calc_x(current_S, desired_S, current_u, desired_u)
        


        if  X < 1: 
            f_b = FRModel.calc_throttle2(current_u, X, a, b, d)
            a_max = Dynamics.get_max_acceleration(current_u,G)
            result =f_b*a_max
        else:
            try:
                collision_avoidance = (current_u**2 - leader_u**2 + math.sqrt((current_u**2 - leader_u**2)**2))**2 / (16*(d_desired + 9.81*G) * (current_S - jam_S + 0.001)**2)
                result = collision_avoidance
            except:
                result = 0
            if np.isnan(result): result = 0
        
        # print(X, f_b, a_max, f_b*a_max, "Current Spacing: ",current_S,"Desired Spacing: ",desired_S, "Current Speed: ",current_u, "Desired Speed: ", desired_u, "ACC: ", result, leader_u)

        if result < 0: result = math.floor(1000*result)/1000

        max_jerk = 1.3
        if prev_a >= 0 and consider_jerk: result = min(result, prev_a + 0.1*max_jerk)
        return result


    def get_a_ub(remaining_distance, time_step, initial_speed):
        return 2*(remaining_distance - initial_speed*time_step)/time_step**2


    def get_throttle_cf(current_u, current_S, leader_u, curr_segment):
        u_f = curr_segment.u_f
        q_c = curr_segment.q_c
        k_j = curr_segment.k_j
        u_c = curr_segment.u_c
        G = curr_segment.grade


        desired_u = FRModel.calc_desired_u(current_S, u_f, q_c, k_j, u_c)
        desired_S = FRModel.calc_desired_S(current_u, current_S, leader_u, u_f, q_c, k_j, u_c)
        X = FRModel.calc_x(current_S, desired_S, current_u, desired_u)
        f_b = FRModel.calc_throttle(current_u, X)

        return max(0,f_b)

# print(calc_desired_S(15, 16))

# print(FRModel.get_a(22.22, 78.6, 7.03, 22.22, 0.52777, 0.16, 16.7))
# print(FRModel.get_a(22.22, 200, 7.03, 22.22, 0.52777, 0.16, 16.7))

# print(FRModel.get_a(0, 20, 7.03, 22.22, 0.52777, 0.16, 16.7))
# print(FRModel.get_a_2(1.047265625, 19.046875, 0.1, 3.05, 10.917, 5.79,17.8,0.5,0.16,0.8*17.8,0))
# print(FRModel.get_a_2(0.1, 10, 0.1, 3.05, 10.917, 5.79,17.8,0.5,0.16,0.8*17.8,0))
# print(FRModel.calc_desired_S(17.8, 'asd', 17.8, 17.8, 0.5 ,0.16, 0.8*17.8))
# print(1 / (0.5/(0.8*17.8)))