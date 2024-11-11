import math
class VanAerdeModel:
    # u_f=16.7, 
    # q_c=1800.0/360
    # k_j=160.0/1000
    # u_c=40.0/3.6
    def calc_van_aerde_parameters(u_f, q_c, k_j, u_c):
        # print(u_f, q_c, k_j, u_c)
        c1 = u_f * (2*u_c - u_f) / (k_j * u_c**2)
        c2 = u_f * (u_f - u_c)**2 / (k_j * u_c**2)
        c3 = (1/q_c) - u_f / (k_j * u_c**2)
        # print(c1, c2, c3)
        return c1, c2, c3

    def calc_va_k_q(u_i, u_f, q_c, k_j, u_c):
        """calculates density and flow for a given speed using Van Aerde Model.

        Args:
            u_i (float): Following vehicle speed value (km/h).
            u_f (float): free flow speed (km/h).
            q_c (float): Basic saturation flow rate per lane (veh/h).
            k_j (float): jam density (veh/km/lane). Defaults to 160.
            u_c (float): speed at capacity (km/h). Defaults to 40.

        Returns:
            tuple: (current densities (veh/km/lane), current flows(veh/h)).
        """
        c1, c2, c3 = VanAerdeModel.calc_van_aerde_parameters(u_f, q_c, k_j, u_c)
        k_i = 1 / (c1 + c3*u_i + (c2 / (u_f - u_i)))
        q_i = k_i*u_i
        return k_i, q_i

    def calc_va_u(spacing_i, u_f, q_c, k_j, u_c):
        """calculate speed for a given spacing using Van Aerde Model.

        Args:
            spacing_i (float): given spacing (1/density).
            u_i (float): speed value (km/h).
            u_f (float): free flow speed (km/h).
            q_c (float): Basic saturation flow rate per lane (veh/h).
            k_j (float): jam density (veh/km/lane). Defaults to 160.
            u_c (float): speed at capacity (km/h). Defaults to 40.

        Returns:
            float: calculated speed
        """
        c1, c2, c3 = VanAerdeModel.calc_van_aerde_parameters(u_f, q_c, k_j, u_c)
        u_i = (-c1 + c3*u_f + spacing_i - math.sqrt((c1-c3*u_f - spacing_i)**2 - 4*c3*((spacing_i-c1)*u_f -c2))) / (2*c3)
        return u_i


# for speed in range(1, 64, 1):
#     q2, k2 = 0, 160
#     k3, q3 = VanAerdeModel.calc_va_k_q(speed, 64, 1800, 160, 0.8*64)
#     print(speed,  k3, q3, (q2-q3)/(k2-k3))

# print('K, q', VanAerdeModel.calc_va_k_q(40, 64, 1800, 160, 0.8*64))
# print(VanAerdeModel.calc_va_u(6.25506437632626, 17.8, 0.5, 0.16, 0.8*17.8))