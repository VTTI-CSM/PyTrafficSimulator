import math
import numpy as np
try: 
    from src.core.models.dynamics import Dynamics
    from src.core.models.va_model import VanAerdeModel
except: 
    from dynamics import Dynamics
    from va_model import VanAerdeModel


class RPAModel:
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
        result = (1/k_j/u_c/u_c) * ((k_j*u_c*u_c/q_c) - u_f + ((u_f - u_c)**2 / u_f / (u_f-current_u)))
        return result
    
    def calc_desired_S(u_np1, u_n, u_f, q_c, k_j, u_c):
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
            tau = abs(RPAModel.calc_tau(u_np1, u_f, q_c, k_j, u_c))
            result = jam_S + tau * u_np1
        else:
            d_desired = -3          #m/s2
            result = max(jam_S, c1 + c2/(u_f - u_n+0.01) + c3*u_n - ((u_np1**2 - u_n**2 - math.sqrt((u_np1**2 - u_n**2)**2)) / (4*d_desired)) ) # ERROR: here 16 should be 4
            
        return result
    

    def steady_state_speed(current_u, leader_u, leader_S, dt, u_f, q_c, k_j, u_c):
        spacing = leader_S
        speed_SA = VanAerdeModel.calc_va_u(spacing, u_f, q_c, k_j, u_c)
        a_SA = (speed_SA - current_u)/dt
        return a_SA

    def collision_avoidance(current_u, leader_u, leader_S, jam_S, u_f, q_c, k_j, u_c, G):
        b_max = -3
        # spacing = RPAModel.calc_desired_S(current_u, leader_u, u_f, q_c, k_j, u_c)
        spacing = leader_S

        d_required = (current_u**2 - leader_u**2 + math.sqrt((current_u**2 - leader_u**2)**2)) / (4 * (max(spacing,jam_S) - jam_S + 0.01))
        collision_avoidance = max(-6,  d_required**2 / (b_max + 9.81*G) )
        return collision_avoidance
    
    
        

    def get_a(current_u, leader_u, leader_S, u_f, q_c, k_j, u_c, dt, G):
        a_SA = RPAModel.steady_state_speed(current_u, leader_u, leader_S, dt, u_f, q_c, k_j, u_c)
        a_CA = RPAModel.collision_avoidance(current_u, leader_u, leader_S, 1/k_j, u_f, q_c, k_j, u_c, G)
        a_dyn = Dynamics.get_max_acceleration(current_u,G)

        if a_CA < 0: a = min(a_CA, a_dyn, a_SA)
        else: a = min(a_dyn, a_SA)

        # a = min(a_CA, a_dyn, a_SA)

        return round(a,3)