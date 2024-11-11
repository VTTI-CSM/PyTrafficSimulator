from src import *

#TODO: later - use the same configurations of integration input files, so that the same model can be tested


lane_width = 3.5
number_of_lanes = 3
intersection_size = 12
link_length = 100
simulation_duration = 3600   #seconds



sim = Simulation(simulation_duration)

#Nodes
sim.create_node({'id': 0, 'coord': ((lane_width*number_of_lanes)/2, link_length+intersection_size*number_of_lanes/2)})
sim.create_node({'id': 1, 'coord': ((lane_width*number_of_lanes)/2, intersection_size*number_of_lanes/2)})
sim.create_node({'id': 2, 'coord': (link_length+intersection_size*number_of_lanes/2, -lane_width*number_of_lanes/2)})
sim.create_node({'id': 3, 'coord': (intersection_size*number_of_lanes/2, -lane_width*number_of_lanes/2)})
sim.create_node({'id': 4, 'coord': (-lane_width*number_of_lanes/2, -link_length-intersection_size*number_of_lanes/2)})
sim.create_node({'id': 5, 'coord': (-lane_width*number_of_lanes/2, -intersection_size*number_of_lanes/2)})
sim.create_node({'id': 6, 'coord': (-link_length-intersection_size*number_of_lanes/2, lane_width*number_of_lanes/2)})
sim.create_node({'id': 7, 'coord': (-intersection_size*number_of_lanes/2, lane_width*number_of_lanes/2)})

sim.create_node({'id': 8, 'coord': (-lane_width*number_of_lanes/2, intersection_size*number_of_lanes/2)})
sim.create_node({'id': 9, 'coord': (-lane_width*number_of_lanes/2, link_length+intersection_size*number_of_lanes/2)})
sim.create_node({'id': 10, 'coord': (intersection_size*number_of_lanes/2, lane_width*number_of_lanes/2)})
sim.create_node({'id': 11, 'coord': (link_length+intersection_size*number_of_lanes/2, lane_width*number_of_lanes/2)})
sim.create_node({'id': 12, 'coord': (lane_width*number_of_lanes/2, -intersection_size*number_of_lanes/2)})
sim.create_node({'id': 13, 'coord': (lane_width*number_of_lanes/2, -link_length-intersection_size*number_of_lanes/2)})
sim.create_node({'id': 14, 'coord': (-intersection_size*number_of_lanes/2, -lane_width*number_of_lanes/2)})
sim.create_node({'id': 15, 'coord': (-link_length-intersection_size*number_of_lanes/2, -lane_width*number_of_lanes/2)})


# print([sim.nodes[i].id for i in sim.nodes.keys()])


# NB, WB, SB, EB
# Intersection in
#hard coding lane striping
# directions = [['LT', 'THR', 'RT']]
# directions = [['LT', 'THR'],['THR'],['THR'],['THR'],['THR'], ['RT', 'THR']]
directions = [['LT', 'THR'],['THR'], ['RT', 'THR']]
# directions = [['LT', 'THR'], ['RT', 'THR']]


seg0 = sim.create_segment({'id': 0, 'start_node': 0, 'end_node': 1, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'directions': directions})
seg1 = sim.create_segment({'id': 1, 'start_node': 2, 'end_node': 3, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'directions': directions})
seg2 = sim.create_segment({'id': 2, 'start_node': 4, 'end_node': 5, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'directions': directions})
seg3 = sim.create_segment({'id': 3, 'start_node': 6, 'end_node': 7, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'directions': directions})

# Intersection out
seg4 = sim.create_segment({'id': 4, 'start_node': 8, 'end_node': 9, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width})
seg5 = sim.create_segment({'id': 5, 'start_node': 10, 'end_node': 11, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width})
seg6 = sim.create_segment({'id': 6, 'start_node': 12, 'end_node': 13, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width})
seg7 = sim.create_segment({'id': 7, 'start_node': 14, 'end_node': 15, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width})

# Straight Connector
seg8 = sim.create_segment({'id': 8, 'start_node': 1, 'end_node': 12, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg9 = sim.create_segment({'id': 9, 'start_node': 3, 'end_node': 14, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg10 = sim.create_segment({'id': 10, 'start_node': 5, 'end_node': 8, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg11 = sim.create_segment({'id': 11, 'start_node': 7, 'end_node': 10, 'number_of_lanes': number_of_lanes, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})


# Right turn Connector
seg12 = sim.create_quadratic_bezier_curve({'id': 12, 'start_link':seg0, 'end_link': seg5, 'order': number_of_lanes-1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg13 = sim.create_quadratic_bezier_curve({'id': 13, 'start_link':seg1, 'end_link': seg6, 'order': number_of_lanes-1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg14 = sim.create_quadratic_bezier_curve({'id': 14, 'start_link':seg2, 'end_link': seg7, 'order': number_of_lanes-1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg15 = sim.create_quadratic_bezier_curve({'id': 15, 'start_link':seg3, 'end_link': seg4, 'order': number_of_lanes-1, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})

# Left turn
seg16 = sim.create_quadratic_bezier_curve({'id': 16, 'start_link':seg0, 'end_link': seg7, 'order': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg17 = sim.create_quadratic_bezier_curve({'id': 17, 'start_link':seg1, 'end_link': seg4, 'order': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg18 = sim.create_quadratic_bezier_curve({'id': 18, 'start_link':seg2, 'end_link': seg5, 'order': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})
seg19 = sim.create_quadratic_bezier_curve({'id': 19, 'start_link':seg3, 'end_link': seg6, 'order': 0, 'lane_width': lane_width, 'has_marking': False, 'link_type': 'connector'})


sim.create_signal([[0], [1], [2], [3]], {'cycle': [(True, False, False, False), 
                                                   (False, True, False, False),
                                                   (False, False, True, False), 
                                                   (False, False, False, True)],
                                         'phase_length': [20, 20, 20, 20]})

# sim.create_signal([[0, 2], [1, 3]], {'cycle': [(True, False, True, False),
#                                                (False, True, False, True)],
#                                      'phase_length': [30, 30]})


# sim.create_signal([[1], [3]])

#creating detectors
sim.create_detector(0, {'id':0, 'x':2, 'length':2, 'pulling_freq': 10})
sim.create_detector(1, {'id':1, 'x':2, 'length':2, 'pulling_freq': 10})
sim.create_detector(2, {'id':2, 'x':2, 'length':2, 'pulling_freq': 10})
sim.create_detector(3, {'id':3, 'x':2, 'length':2, 'pulling_freq': 10})

vg1 = VehicleGenerator(sim, {
    'vehicles': [
        (100, {'start_node': 0, 'end_node': 13, 'v_0': 16.6, 'veh_class':1}),    #NB-THR
        (100, {'start_node': 0, 'end_node': 11, 'v_0': 16.6, 'veh_class':1}),   #NB-RT
        (100, {'start_node': 0, 'end_node': 15, 'v_0': 16.6, 'veh_class':1})   #NB-LT
        ]})

# vg1 = VehicleGenerator(sim, {
#     'vehicles': [
#         (100, {'start_node': 0, 'end_node': 13, 'v_0': 16.6, 'veh_class':1}),    #NB-THR
#         (100, {'start_node': 0, 'end_node': 11, 'v_0': 16.6, 'veh_class':1}),   #NB-RT
#         (100, {'start_node': 0, 'end_node': 15, 'v_0': 16.6, 'veh_class':1})   #NB-LT
#         ]})

vg2 = VehicleGenerator(sim, {
    'vehicles': [
        (100, {'start_node': 2, 'end_node': 15, 'v_0': 16.6, 'veh_class':1}),    #NB-THR
        (100, {'start_node': 2, 'end_node': 13, 'v_0': 16.6, 'veh_class':1}),   #NB-RT
        (100, {'start_node': 2, 'end_node': 9, 'v_0': 16.6, 'veh_class':1})   #NB-LT
        ]})


vg3 = VehicleGenerator(sim, {
    'vehicles': [
        (100, {'start_node': 4, 'end_node': 9, 'v_0': 16.6, 'veh_class':1}),    #NB-THR
        (100, {'start_node': 4, 'end_node': 15, 'v_0': 16.6, 'veh_class':1}),   #NB-RT
        (100, {'start_node': 4, 'end_node': 11, 'v_0': 16.6, 'veh_class':1})   #NB-LT
        ]})


vg4 = VehicleGenerator(sim, {
    'vehicles': [
        (100, {'start_node': 6, 'end_node': 11, 'v_0': 16.6, 'veh_class':1}),    #NB-THR
        (100, {'start_node': 6, 'end_node': 9, 'v_0': 16.6, 'veh_class':1}),   #NB-RT
        (100, {'start_node': 6, 'end_node': 13, 'v_0': 16.6, 'veh_class':1})   #NB-LT
        ]})


sim.add_vehicle_generator(vg1)
sim.add_vehicle_generator(vg2)
sim.add_vehicle_generator(vg3)
sim.add_vehicle_generator(vg4)



v1 = Vehicle({'path': [0, 8, 6], 'x': 0, 'v_0':16.6})
v2 = Vehicle({'path': [0, 12, 5], 'x': 10, 'v_0':16.6})
v3 = Vehicle({'path': [0, 16, 7], 'x': 20, 'v_0':16.6})
# sim.add_vehicle(v1)
# sim.add_vehicle(v2)
# sim.add_vehicle(v3)

# v = Vehicle({'path': [0]})
# sim.add_vehicle(v)

win = Window(sim)
win.initialize()
win.show(show=True)
