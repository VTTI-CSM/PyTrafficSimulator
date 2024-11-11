import sys
import os
import pandas as pd
import numpy as np
dir = os.path.abspath('')
dir = dir[slice(0,dir.find('PyTrafficSimulator') + len('PyTrafficSimulator'),1)]
sys.path.append(dir)
from src import *
sys.dont_write_bytecode = True
# Random seed exisits in the vehicle, vehicle generator and the nb_controller.

lane_width = 3.2                # meters approx 12 ft
simulation_duration = 4000      # seconds after which it will stop (s)
demand_end_time = 3600
random_seed = 10
initial_simulation_speed = 50
avg_veh_length = 5.0 #meters


sim = Simulation(simulation_duration, random_seed)

#Nodes
add_l = -5
sim.create_node({'id': 1, 'coord': (10, 75+add_l)})
sim.create_node({'id': 2, 'coord': (10, 28)})
sim.create_node({'id': 3, 'coord': (45+add_l, -8)})
sim.create_node({'id': 4, 'coord': (30, -8)})
sim.create_node({'id': 5, 'coord': (-10, -80-add_l)})
sim.create_node({'id': 6, 'coord': (-10, -28)})
sim.create_node({'id': 7, 'coord': (-47-add_l, 8)})
sim.create_node({'id': 8, 'coord': (-30, 8)})

sim.create_node({'id': 9, 'coord': (-10, 28)})
sim.create_node({'id': 10, 'coord': (-10, 75+add_l)})
sim.create_node({'id': 11, 'coord': (30, 8)})
sim.create_node({'id': 12, 'coord': (45+add_l, 8)})
sim.create_node({'id': 13, 'coord': (10, -28)})
sim.create_node({'id': 14, 'coord': (10, -80-add_l)})
sim.create_node({'id': 15, 'coord': (-30, -8)})
sim.create_node({'id': 16, 'coord': (-47-add_l, -8)})


u_f, u_c, q_c, k_j = 22.35, 0.5*22.35, 1800, 1000/8.76

# Intersection in
seg1 = sim.create_segment({'id': 1, 'start_node': 1, 'end_node': 2, 'number_of_lanes': 6, 'lane_width': lane_width, 'directions': [['LT'], ['LT'], ['THR'], ['THR'], ['THR'], ['RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg2 = sim.create_segment({'id': 2, 'start_node': 3, 'end_node': 4, 'number_of_lanes': 5, 'lane_width': lane_width, 'directions': [['LT'], ['LT'], ['THR'], ['THR'], ['THR', 'RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg3 = sim.create_segment({'id': 3, 'start_node': 5, 'end_node': 6, 'number_of_lanes': 6, 'lane_width': lane_width, 'directions': [['LT'], ['LT'], ['THR'], ['THR'], ['THR'], ['RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg4 = sim.create_segment({'id': 4, 'start_node': 7, 'end_node': 8, 'number_of_lanes': 6, 'lane_width': lane_width, 'directions': [['LT'], ['LT'], ['THR'], ['THR'], ['THR'], ['RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Intersection out
seg5 = sim.create_segment({'id': 5, 'start_node': 9, 'end_node': 10, 'number_of_lanes': 3, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg6 = sim.create_segment({'id': 6, 'start_node': 11, 'end_node': 12, 'number_of_lanes': 3, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg7 = sim.create_segment({'id': 7, 'start_node': 13, 'end_node': 14, 'number_of_lanes': 3, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg8 = sim.create_segment({'id': 8, 'start_node': 15, 'end_node': 16, 'number_of_lanes': 3, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Straight Connector
seg9 = sim.create_straight_conn({'id': 9, 'start_link':seg1, 'end_link': seg7, 'number_of_lanes': 3, 'order_start': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg10 = sim.create_straight_conn({'id': 10, 'start_link':seg2, 'end_link': seg8, 'number_of_lanes': 3, 'order_start': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg11 = sim.create_straight_conn({'id': 11, 'start_link':seg3, 'end_link': seg5, 'number_of_lanes': 3, 'order_start': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg11 = sim.create_straight_conn({'id': 12, 'start_link':seg4, 'end_link': seg6, 'number_of_lanes': 3, 'order_start': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})



# Right turn Connector
seg13 = sim.create_quadratic_bezier_curve({'id': 13, 'start_link':seg1, 'end_link': seg6, 'order_start': 5, 'order_end': 2, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg14 = sim.create_quadratic_bezier_curve({'id': 14, 'start_link':seg2, 'end_link': seg7, 'order_start': 4, 'order_end': 2, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg15 = sim.create_quadratic_bezier_curve({'id': 15, 'start_link':seg3, 'end_link': seg8, 'order_start': 5, 'order_end': 2, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg16 = sim.create_quadratic_bezier_curve({'id': 16, 'start_link':seg4, 'end_link': seg5, 'order_start': 5, 'order_end': 2, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Left turn
seg17 = sim.create_quadratic_bezier_curve({'id': 17, 'start_link':seg1, 'end_link': seg8, 'order_start': 0, 'number_of_lanes': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg18 = sim.create_quadratic_bezier_curve({'id': 18, 'start_link':seg2, 'end_link': seg5, 'order_start': 0, 'number_of_lanes': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg19 = sim.create_quadratic_bezier_curve({'id': 19, 'start_link':seg3, 'end_link': seg6, 'order_start': 0, 'number_of_lanes': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg20 = sim.create_quadratic_bezier_curve({'id': 20, 'start_link':seg4, 'end_link': seg7, 'order_start': 0, 'number_of_lanes': 2, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})




# Optimal FT  timing
volume_factor = 1.0
# [SBL, NBT, WBT, EBL, SBT, NBL, WBL, EBT]
max_green = [19, 86, 60, 46, 55, 38, 65, 71]
min_green = [5, 15, 15, 5, 15, 5, 5, 15]

# MAH = 4
# Lv, Ld = 20, 35
# vs = 45
# passage_time = MAH - (Lv + Ld)/(1.47*vs)

sim.create_signal_nema({'id': 1,
                        'lanes': [
                            seg3.lanes[0:2],
                            seg1.lanes[2:6],
                            seg2.lanes[2:5],
                            seg4.lanes[0:2],
                            seg3.lanes[2:6],
                            seg1.lanes[0:2],
                            seg2.lanes[0:2],
                            seg4.lanes[2:6]],
                        'actuated': True,
                        'yellow_time': 5,
                        'all_red_time': 1,
                        'max_green': max_green,
                        'min_green': min_green,
                        'passage_time': 3.1            # Equation 5-5 / Table 5-10
                        })

#creating detectors
sim.create_detector(1, {'id':1000, 'type':20, 'x':42+add_l, 'length':10, 'pulling_freq': 30})
sim.create_detector(2, {'id':2000, 'type':20, 'x':10+add_l, 'length':10, 'pulling_freq': 30})
sim.create_detector(3, {'id':3000, 'type':20, 'x':47+add_l, 'length':10, 'pulling_freq': 30})
sim.create_detector(4, {'id':4000, 'type':20, 'x':12+add_l, 'length':10, 'pulling_freq': 30})



print('Reading Vehicle Demand...')
df = pd.read_csv(dir + r'\examples\Orlando_veh_input.csv')
direction_dict = {'NB_THR': {'start_node':1, 'end_node':14, 'veh_class':1},
                  'NB_RT': {'start_node':1, 'end_node':12, 'veh_class':1},
                  'NB_LT': {'start_node':1, 'end_node':16, 'veh_class':1},
                   
                  'SB_THR': {'start_node':5, 'end_node':10, 'veh_class':1},
                  'SB_RT': {'start_node':5, 'end_node':16, 'veh_class':1},
                  'SB_LT': {'start_node':5, 'end_node':12, 'veh_class':1},

                  'WB_THR': {'start_node':3, 'end_node':16, 'veh_class':1},
                  'WB_RT': {'start_node':3, 'end_node':14, 'veh_class':1},
                  'WB_LT': {'start_node':3, 'end_node':10, 'veh_class':1},

                  'EB_THR': {'start_node':7, 'end_node':12, 'veh_class':1},
                  'EB_RT': {'start_node':7, 'end_node':10, 'veh_class':1},
                  'EB_LT': {'start_node':7, 'end_node':14, 'veh_class':1}
                  }

# average each dt seconds
dt = 5
tot_vol = 0
for t in np.arange(0,3600,dt):
    dir_list = list(df[(df.T0 >= t) & (df.T0 < t+dt)].Direction)

    volume_dict = {'NB_THR': dir_list.count('NB_THR')*3600/dt,
                   'NB_RT': dir_list.count('NB_RT')*3600/dt,
                   'NB_LT': dir_list.count('NB_LT')*3600/dt,
                   'SB_THR': dir_list.count('SB_THR')*3600/dt,
                   'SB_RT': dir_list.count('SB_RT')*3600/dt,
                   'SB_LT': dir_list.count('SB_LT')*3600/dt,
                   'WB_THR': dir_list.count('WB_THR')*3600/dt,
                   'WB_RT': dir_list.count('WB_RT')*3600/dt,
                   'WB_LT': dir_list.count('WB_LT')*3600/dt,
                   'EB_THR': dir_list.count('EB_THR')*3600/dt,
                   'EB_RT': dir_list.count('EB_RT')*3600/dt,
                   'EB_LT': dir_list.count('EB_LT')*3600/dt}

    
    for direction in volume_dict.keys():
        if volume_dict[direction] > 0:
            if 'RT' in direction:
                rt_factor = 0.2
            else: rt_factor = 1.0
            if volume_factor*volume_dict[direction]*rt_factor >= 1:
                vg = VehicleGenerator(sim, {
                'start_time': t,
                'end_time': min(t+dt-1, demand_end_time),
                'arrival_type': 'uniform',
                'vehicles': [
                    (volume_factor*volume_dict[direction]*rt_factor,
                     {'start_node': direction_dict[direction]['start_node'],
                       'end_node': direction_dict[direction]['end_node'],
                        'veh_class':direction_dict[direction]['veh_class'],
                        'l': avg_veh_length
                        })]})
                sim.add_vehicle_generator(vg)
                tot_vol += volume_factor*volume_dict[direction]*rt_factor * dt/3600


win = Window(sim, initial_simulation_speed)
win.initialize(is_running=True)
win.show(show=True)