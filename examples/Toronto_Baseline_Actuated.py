import sys
import os
dir = os.path.abspath('')
dir = dir[slice(0,dir.find('PyTrafficSimulator') + len('PyTrafficSimulator'),1)]
sys.path.append(dir)
from src import *
sys.dont_write_bytecode = True
# Random seed exisits in the vehicle, vehicle generator and the nb_controller.

lane_width = 3.65               # meters approx 12 ft
simulation_duration = 4000      # seconds after which it will stop (s)
random_seed = 100


sim = Simulation(simulation_duration, random_seed)

#Nodes
sim.create_node({'id': 1, 'coord': (3.65, 359)})
sim.create_node({'id': 2, 'coord': (3.65, 9.125+10)})
sim.create_node({'id': 3, 'coord': (249, -3.65)})
sim.create_node({'id': 4, 'coord': (9.125+10, -3.65)})
sim.create_node({'id': 5, 'coord': (-3.65, -190)})
sim.create_node({'id': 6, 'coord': (-3.65, -9.125-10)})
sim.create_node({'id': 7, 'coord': (-200, 3.65)})
sim.create_node({'id': 8, 'coord': (-9.125-10, 3.65)})

sim.create_node({'id': 9, 'coord': (-5.475, 9.125+10)})
sim.create_node({'id': 10, 'coord': (-5.475, 359)})
sim.create_node({'id': 11, 'coord': (9.125+10, 5.475)})
sim.create_node({'id': 12, 'coord': (249, 5.475)})
sim.create_node({'id': 13, 'coord': (5.475, -9.125-10)})
sim.create_node({'id': 14, 'coord': (5.475, -190)})
sim.create_node({'id': 15, 'coord': (-9.125-10, -5.475)})
sim.create_node({'id': 16, 'coord': (-200, -5.475)})


u_f, u_c, q_c, k_j = 11.1, 8.33, 1900, 160

# Intersection in
seg1 = sim.create_segment({'id': 1, 'start_node': 1, 'end_node': 2, 'number_of_lanes': 3, 'lane_width': lane_width, 'directions': [['LT'], ['THR'], ['THR', 'RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg2 = sim.create_segment({'id': 2, 'start_node': 3, 'end_node': 4, 'number_of_lanes': 3, 'lane_width': lane_width, 'directions': [['LT'], ['THR'], ['THR', 'RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg3 = sim.create_segment({'id': 3, 'start_node': 5, 'end_node': 6, 'number_of_lanes': 3, 'lane_width': lane_width, 'directions': [['LT'], ['THR'], ['THR', 'RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg4 = sim.create_segment({'id': 4, 'start_node': 7, 'end_node': 8, 'number_of_lanes': 3, 'lane_width': lane_width, 'directions': [['LT'], ['THR'], ['THR', 'RT']], 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Intersection out
seg5 = sim.create_segment({'id': 5, 'start_node': 9, 'end_node': 10, 'number_of_lanes': 2, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg6 = sim.create_segment({'id': 6, 'start_node': 11, 'end_node': 12, 'number_of_lanes': 2, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg7 = sim.create_segment({'id': 7, 'start_node': 13, 'end_node': 14, 'number_of_lanes': 2, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg8 = sim.create_segment({'id': 8, 'start_node': 15, 'end_node': 16, 'number_of_lanes': 2, 'lane_width': lane_width, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Straight Connector
seg9 = sim.create_straight_conn({'id': 9, 'start_link':seg1, 'end_link': seg7, 'number_of_lanes': 2, 'order_start': 1, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg10 = sim.create_straight_conn({'id': 10, 'start_link':seg2, 'end_link': seg8, 'number_of_lanes': 2, 'order_start': 1, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg11 = sim.create_straight_conn({'id': 11, 'start_link':seg3, 'end_link': seg5, 'number_of_lanes': 2, 'order_start': 1, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg11 = sim.create_straight_conn({'id': 12, 'start_link':seg4, 'end_link': seg6, 'number_of_lanes': 2, 'order_start': 1, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})



# Right turn Connector
seg13 = sim.create_quadratic_bezier_curve({'id': 13, 'start_link':seg1, 'end_link': seg6, 'order_start': 2, 'order_end': 1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg14 = sim.create_quadratic_bezier_curve({'id': 14, 'start_link':seg2, 'end_link': seg7, 'order_start': 2, 'order_end': 1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg15 = sim.create_quadratic_bezier_curve({'id': 15, 'start_link':seg3, 'end_link': seg8, 'order_start': 2, 'order_end': 1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg16 = sim.create_quadratic_bezier_curve({'id': 16, 'start_link':seg4, 'end_link': seg5, 'order_start': 2, 'order_end': 1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})

# Left turn
seg17 = sim.create_quadratic_bezier_curve({'id': 17, 'start_link':seg1, 'end_link': seg8, 'order_start': 0, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg18 = sim.create_quadratic_bezier_curve({'id': 18, 'start_link':seg2, 'end_link': seg5, 'order_start': 0, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg19 = sim.create_quadratic_bezier_curve({'id': 19, 'start_link':seg3, 'end_link': seg6, 'order_start': 0, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})
seg20 = sim.create_quadratic_bezier_curve({'id': 20, 'start_link':seg4, 'end_link': seg7, 'order_start': 0, 'order_end': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector', 'u_f': u_f, 'u_c': u_c, 'q_c': q_c, 'k_j':k_j})


pre_timed_timing = [12, 54, 24, 61, 24, 43, 32, 55] 
max_green = [1.0*(x-3) for x in pre_timed_timing]
min_green = [5, 15, 5, 15, 5, 15, 5, 15]
sim.create_signal_nema({'id': 1,
                        'lanes': [[seg1.lanes[0]],
                                  seg3.lanes[1:3],
                                  [seg4.lanes[0]],
                                  seg2.lanes[1:3],
                                  [seg3.lanes[0]],
                                  seg1.lanes[1:3],
                                  [seg2.lanes[0]],
                                  seg4.lanes[1:3]],
                        'actuated': True,
                        'yellow_time': 3.0,
                        'all_red_time': 1.0,
                        'max_green': max_green,
                        'min_green': min_green,
                        'passage_time': 1.3            # Equation 5-5 
                        })

#creating detectors
sim.create_detector(1, {'id':1000, 'type':20, 'x':327, 'length':24, 'pulling_freq': 30})
sim.create_detector(2, {'id':2000, 'type':20, 'x':217, 'length':24, 'pulling_freq': 30})
sim.create_detector(3, {'id':3000, 'type':20, 'x':158, 'length':24, 'pulling_freq': 30})
sim.create_detector(4, {'id':4000, 'type':20, 'x':168, 'length':24, 'pulling_freq': 30})


volume_factor = 1.0
start_time = 0
end_time = 3600
vg1 = VehicleGenerator(sim, {
    'start_time': start_time,
    'end_time': end_time,
    'vehicles': [
        (volume_factor*721, {'start_node': 1, 'end_node': 14, 'veh_class':1}),      #NB-THR
        (volume_factor*88, {'start_node': 1, 'end_node': 12, 'veh_class':1}),       #NB-RT
        (volume_factor*71, {'start_node': 1, 'end_node': 16, 'veh_class':1})        #NB-LT
        ]})

vg2 = VehicleGenerator(sim, {
    'start_time': start_time,
    'end_time': end_time,
    'vehicles': [
        (volume_factor*844, {'start_node': 3, 'end_node': 16, 'veh_class':1}),      #WB-THR
        (volume_factor*86, {'start_node': 3, 'end_node': 14, 'veh_class':1}),       #WB-RT
        (volume_factor*278, {'start_node': 3, 'end_node': 10, 'veh_class':1})       #WB-LT
        ]})

vg3 = VehicleGenerator(sim, {
    'start_time': start_time,
    'end_time': end_time,
    'vehicles': [
        (volume_factor*806, {'start_node': 5, 'end_node': 10, 'veh_class':1}),      #SB-THR
        (volume_factor*100, {'start_node': 5, 'end_node': 16, 'veh_class':1}),      #SB-RT
        (volume_factor*188, {'start_node': 5, 'end_node': 12, 'veh_class':1})       #SB-LT
        ]})

vg4 = VehicleGenerator(sim, {
    'start_time': start_time,
    'end_time': end_time,
    'vehicles': [
        (volume_factor*1223, {'start_node': 7, 'end_node': 12, 'veh_class':1}),     #EB-THR
        (volume_factor*121, {'start_node': 7, 'end_node': 10, 'veh_class':1}),      #EB-RT
        (volume_factor*134, {'start_node': 7, 'end_node': 14, 'veh_class':1})       #EB-LT
        ]})


sim.add_vehicle_generator(vg1)
sim.add_vehicle_generator(vg2)
sim.add_vehicle_generator(vg3)
sim.add_vehicle_generator(vg4)

win = Window(sim)
win.initialize(is_running=True)
win.show(show=True)