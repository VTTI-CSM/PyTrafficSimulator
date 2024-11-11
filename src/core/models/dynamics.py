import pandas as pd
import os

dir = os.path.abspath('')
dir = dir[slice(0,dir.find('PyTrafficSimulator') + len('PyTrafficSimulator'),1)]


class Dynamics:
    data = pd.read_csv(dir + r'\src\core\vehicles_parameters.csv', index_col=0)

    # selected_veh_make = '2018 Toyota Camry LE2.5'
    selected_veh_make = 'Cadillac 2014 SRX'
    # selected_veh_make = 'Toyota RAV 4'

    alpha0 = data['alpha0'][selected_veh_make]
    alpha1 = data['alpha1'][selected_veh_make]
    alpha2 = data['alpha2'][selected_veh_make]
    C_r = data['Cr'][selected_veh_make]                      #Rolling Coefficients
    C_1 = data['C1'][selected_veh_make]
    C_2 = data['C2'][selected_veh_make]
    C_d = data['Cd'][selected_veh_make]                      # drag coefficient
    A_f = data['Af'][selected_veh_make]                      #Frontal Area (m2) = 0.85 * height * width ***************************
    eff = data['eff'][selected_veh_make]                     # Engine Efficiency
    pta = data['pta'][selected_veh_make]                     # percentage mass on tractive axle
    m = data['m'][selected_veh_make]                         #Vehicle Mass (kg)
    max_power = data['max_power'][selected_veh_make]         #kW
    C_h = data['Ch'][selected_veh_make]                      #altiturde corr, computed as 1–0.085H where H is the altitude (km) (blacksburg VA) - #set 0.95 from rakha 2004 doi:10.3141/1883-05
    rho = data['rho'][selected_veh_make]                     #Airdyn (kg/m3)
    myo = 0.5                                                # coeff of friction between tires and pavement #set from rakha 2004 doi:10.3141/1883-05
    m_ta = pta*m                                            # mass on tractive axle


    def get_max_throttle(v):
        if v == 0:
            # v = Dynamics.get_speed_threshold()
            return 1
        nyo = 0.92                                                  # transmission (driveline) efficiency #set temporarily
        m_ta = Dynamics.pta*Dynamics.m                              # mass on tractive axle                                                   # coeff of friction between tires and pavement #set from rakha 2004 doi:10.3141/1883-05
        beta = 1

        tf_ub = m_ta*9.8066*Dynamics.myo
        v_threshold = Dynamics.get_speed_threshold()

        if v < v_threshold: tf = 3600*beta*nyo*Dynamics.max_power/v_threshold
        else: tf = 3600*beta*nyo*Dynamics.max_power/v            # here v is in m/s, (v*3.6) is km/hr

        max_throttle = min(1, tf_ub/tf)

        return max_throttle

    
    def get_speed_threshold():
        nyo = 0.92                                                  # transmission (driveline) efficiency #set temporarily
        m_ta = Dynamics.pta*Dynamics.m                              # mass on tractive axle
                                                           # coeff of friction between tires and pavement #set from rakha 2004 doi:10.3141/1883-05
        beta = 1

        v = 3600*beta*nyo*Dynamics.max_power/(m_ta*9.8066*Dynamics.myo)
        return v


    def traction_force(v):
        """
        Computes the instantaneous vehicle traction force given the power, vehicle current velocity, and throttle level. 
        Inputs:
            param power: the instantaneous vehicle power in kW.
            param v: the instantaneous vehicle speed in m/s.
            param throttle: driver throttle input from 0 to 1.
        Returns:
            ##.
        """
        nyo = 0.92                                                  # transmission (driveline) efficiency #set temporarily
        m_ta = Dynamics.pta*Dynamics.m                              # mass on tractive axle
        beta = 1                                                    # set = 1 from veh dynamics paper for light duty vehicles


        tf = 3600*beta*nyo*Dynamics.max_power/(v+0.01)           # here v is in m/s, (v*3.6) is km/hr
        tf_ub = m_ta*9.8066*Dynamics.myo


        return min(tf,tf_ub)


    # def Resistance_force(v, g):
    #     """ Computes the instantaneous vehicle Resistnce force.
    #     Args:
    #         v (float): the instantaneous vehicle speed in m/s
    #     Returns:
    #         float: The instantaneous vehicle Resistnce force should be in Newtons.
    #     """
        
    #     ## using 2011 Honda Accord (Full-size)
    #     #grade: uphill +ve, downhill -ve.
    #     return (Dynamics.rho/25.92)*Dynamics.C_d*Dynamics.C_h*Dynamics.A_f*(v*3.6)**2 + 9.8067*Dynamics.m*(Dynamics.C_r/1000)*(Dynamics.C_1*v*3.6+Dynamics.C_2) + 9.8067*Dynamics.m*g

    def Resistance_force(v, g):
        """
        Computes the instantaneous vehicle resistance force.
        
        Args:
            v (float): Instantaneous vehicle speed in m/s.
            g (float): Road grade (uphill +ve, downhill -ve).
            
        Returns:
            float: Instantaneous vehicle resistance force in Newtons.
        """
        v_kmh = v * 3.6  # Convert speed to km/h
        
        # Precompute repeated constants and terms
        rho_factor = Dynamics.rho / 25.92
        drag_term = rho_factor * Dynamics.C_d * Dynamics.C_h * Dynamics.A_f * v_kmh**2
        rolling_term = 9.8067 * Dynamics.m * (Dynamics.C_r / 1000) * (Dynamics.C_1 * v_kmh + Dynamics.C_2)
        gravity_term = 9.8067 * Dynamics.m * g
        
        return drag_term + rolling_term + gravity_term


    def get_max_acceleration(v, g):
        F_t = Dynamics.traction_force(v)
        R_t = Dynamics.Resistance_force(v, g)
        a = (F_t - R_t)/Dynamics.m
        return a
    
    def get_acceleration(v, g):
        F_t = Dynamics.traction_force(v)
        R_t = Dynamics.Resistance_force(v, g)
        a = (F_t - R_t)/Dynamics.m
        return a
    

    def get_throttle_from_acc(v, acc, g):
        R_t = Dynamics.Resistance_force(v, g)
        F_t = (acc*Dynamics.m) + R_t

        max_power = 132.0                                                   #kW
        v = max(v, 1)
        nyo = 0.92                                                   # transmission (driveline) efficiency #set temporarily
        m_ta = Dynamics.pta*Dynamics.m                              # mass on tractive axle
        beta = 1                                                    #set = 1 from veh dynamics paper for light duty vehicles
        tf = 3600*beta*nyo*max_power/(v)
        tf_ub = m_ta*9.8066*Dynamics.myo

        return max(0, F_t/min(tf,tf_ub))


    # hey = get_accel(,1.0,0)
    # print(hey)
    # print(get_throttle_from_acc(10,hey,0))
