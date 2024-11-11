import numpy as np
try: from src.core.models.dynamics import Dynamics
except: from dynamics import Dynamics
class FCModel:
    def FC(power):
        """
        Computes the fuel consumption based on CPFM-1 Model
        Inputs:
            param power: the instantaneous vehicle power in kW.
        Returns:
            The instantaneous fuel consumption in liter per second.
        """
        if power>=0:
            return Dynamics.alpha0 + Dynamics.alpha1 * power + Dynamics.alpha2 * power**2
        else:
            return Dynamics.alpha0


    def power(a, v, g):
        """
        Computes the power based on CPFM-1 Model.
        Inputs:
            param R: the instantaneous vehicle Resistnce forces.
            param a: the instantaneous vehicle acceleration in m/s2.
            param v: the instantaneous vehicle speed in m/s.
        Returns:
            The instantaneous power in kW.
        """
        R = Dynamics.Resistance_force(v, g)
        P = (v*3.6)*( R + (1.00*Dynamics.m*a))/(3600*Dynamics.eff)
        return min(P, Dynamics.max_power) #kW


    def fc_main(a, v, g):
        "result in L"
        p = FCModel.power(a, v, g)
        F_C = FCModel.FC(p)
        return F_C
    

    def calculate_fc_variable_v(v0, vf, t_tot, gr, dt=1):
        """Calculate fuel consumption for variable speed profile.

        Args:
            v0 (float): initial speed (m/s).
            vf (float): final speed (m/s).
            t_tot (float): total travel time (s).
            dt (float, optional): resolution. Defaults to 0.1.

        Returns:
            float: Fuel consumption in mL
        """
        a = (vf - v0) / t_tot
        v = v0
        fc_tot = 0
        for t in np.arange(dt, t_tot+dt, dt):
            v_prev = v
            v = v + a*dt
            fc_tot += FCModel.fc_main(a, (v_prev + v)/2, gr)*dt*1000
        return fc_tot
    


# input = []
# results = []
# throttle = 0.3
# for v0 in np.arange(0, 17.8, 0.1):
#     accel = throttle * Dynamics.get_max_acceleration(v0, 0)
#     # accel = throttle * Dynamics.get_acceleration(v0, 0)
    
#     ta = (17.8 - v0) / accel
#     xa = (17.8**2 - v0**2) / (2*accel)
#     xc = 180 - xa
#     tc = xc / 17.8

#     input.append(v0)

#     results.append(FCModel.fc_main(accel, (v0+17.8)/2, 0)*ta*1000 + FCModel.fc_main(0, 17.8, 0)*tc*1000)
#     # results.append(Dynamics.get_max_throttle(v0))
#     # results.append(accel)

#     # results.append(Dynamics.traction_force(v0))
#     # results.append(Dynamics.Resistance_force(v0, 0))
#     # results.append(Dynamics.traction_force(v0) - Dynamics.Resistance_force(v0, 0))

# # v0 = 0
# # Dynamics.get_max_throttle(v0)
# # for throttle in np.arange(0, Dynamics.get_max_throttle(v0)+0.1, 0.1):
# #     accel = Dynamics.get_accel(v0, throttle, 0)
# #     input.append(throttle)
# #     results.append(accel)


# from matplotlib import pyplot as plt
# plt.plot(input, results)
# # plt.xlabel('Downstream Initial Speed (m/s)')
# # plt.ylabel('Downstream Fuel Consumption (mL)')
# # plt.title('Fuel Consumption vs Initial Speed')
# plt.grid()
# plt.show()

# print(FCModel.fc_main(0, 18.561000, -0.03)*1000)
# print(FCModel.fc_main(0, 18.561000, 0.03)*1000)
# print(Dynamics.Resistance_force(5, 0.03))
# print(FCModel.power(1.28888888888889, 9.44/3.6, 0))


# dt = 0.1
# v0 = 0
# vf = 20
# t_tot = 20
# a = (vf - v0) / t_tot
# v = v0

# fc_tot = 0
# for t in np.arange(dt, t_tot+dt, dt):
#     v_prev = v
#     v = v + a*dt
#     fc_tot += FCModel.fc_main(a, (v_prev + v)/2, 0)*dt*1000

# fc_heu1 = fc_tot - FCModel.fc_main(a, (v0+vf)/2, 0)*t_tot*1000
# fc_heu2 = fc_tot - (FCModel.fc_main(a, v0, 0) + FCModel.fc_main(a, vf, 0))*0.5*t_tot*1000
# fc_heu3 = fc_tot - (FCModel.fc_main(a, v0, 0) + FCModel.fc_main(a, (v0+vf)/2, 0) + FCModel.fc_main(a, vf, 0))/3*t_tot*1000
# print(fc_tot, fc_heu1, fc_heu2, fc_heu3)
