import numpy as np
from collections import deque

class Lane:
    instances = []
    def __init__(self, sim, config={}):
        # Set default configuration
        self.set_default_config(sim)

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties(sim)

    def set_default_config(self, sim):
        self.simulation = sim  
        self.id = None
        self.points = [(None, None), (None, None)]      #start and end points of the lane
        self.segment = None                             #segment object to which the lane belongs
        self.order = 0                                  #order of the lane - note lanes are ordered consecutively from 0 in the median lane, increasing to the shoulder lane.
        self.direction = ['THR', 'RT', 'LT']
        self.vehicles = deque()
        self.start_node = None
        self.end_node = None
        self.angle = None
        self.distance = None
        self.has_traffic_signal = False
        self.traffic_signal = None
        self.traffic_signal_phase = []
        self.traffic_signal_ring_no = 0
        self.ts_det = None
        self.ts_stoch = None
        self.last_red_indication_t = -99999
        self.last_added_time = 0        # used in vehicle generators
        self.detector = None


    def init_properties(self, sim):
        self.set_functions(sim)


    def set_functions(self, sim):
        self.angle = self.segment.angle
        self.distance = self.segment.length
        self.calculate_coordinates(sim)     
    
    def add_vehicle(self, veh):
        self.vehicles.append(veh)

    def remove_vehicle(self, veh):
        self.vehicles.remove(veh)
    
    def set_traffic_signal(self, signal, phase_no):
        self.traffic_signal = signal
        self.traffic_signal_phase.append(phase_no)
        self.has_traffic_signal = True

        if self.traffic_signal_phase[0] < 4 and self.traffic_signal.NEMA == True:
            self.traffic_signal_ring_no = 0
        elif self.traffic_signal.NEMA == True:
            self.traffic_signal_ring_no = 1
    
    @property
    def traffic_signal_state(self):
        if self.has_traffic_signal:
            i = self.traffic_signal_phase
            for i in self.traffic_signal_phase:
                # i = self.traffic_signal.current_phase_index
                if self.traffic_signal.current_cycle[i] == True:
                    curr_t_phase = self.simulation.t - self.traffic_signal.prev_phase_start[self.traffic_signal_ring_no]
                    # curr_t_phase = self.simulation.t - self.traffic_signal.phase_start
                    
                    if self.traffic_signal.sig_stat[i] == 'r':
                        return 'red'
                    elif self.traffic_signal.sig_stat[i] == 'y':
                        return 'yellow'
                    else:
                        return 'green'

                    # if (curr_t_phase > self.traffic_signal.phase_length[i] - self.traffic_signal.all_red_time) and self.traffic_signal.switch == 1:
                    #     return 'red'
                    # elif (curr_t_phase > self.traffic_signal.phase_length[i] - self.traffic_signal.yellow_time - self.traffic_signal.all_red_time) and self.traffic_signal.switch == 1:
                    #     return 'yellow'
                    # else: 
                    #     return 'green'
                else: pass
        return 'red'
    
    def future_traffic_signal_state(self, time):
        if self.has_traffic_signal:
            i = self.traffic_signal_phase
            for i in self.traffic_signal_phase:
                # i = self.traffic_signal.current_phase_index
                if self.traffic_signal.current_cycle[i] == True:
                    curr_t_phase = time - self.traffic_signal.prev_phase_start[self.traffic_signal_ring_no]
                    # curr_t_phase = self.simulation.t - self.traffic_signal.phase_start

                    if (curr_t_phase > self.traffic_signal.phase_length[i] - self.traffic_signal.all_red_time) and self.traffic_signal.switch == 1:
                        return 'red'
                    elif (curr_t_phase > self.traffic_signal.phase_length[i] - self.traffic_signal.yellow_time - self.traffic_signal.all_red_time) and self.traffic_signal.switch == 1:
                        return 'yellow'
                    else: 
                        return 'green'
                else: pass
        return 'red'

    def calculate_coordinates(self, sim):
        if self.segment.link_type == 'link' or self.segment.link_type == 'StrConn':
            positive_angle = np.rad2deg(self.segment.angle)
            if positive_angle < 0:
                positive_angle = 360 + positive_angle
            

            if (positive_angle >= 45 and positive_angle <= 135) or (positive_angle >= 225 and positive_angle <= 315):
                # vertical link
                o = (round(self.segment.points[0][0] + np.sin(self.segment.angle) * self.segment.link_width/2,3),
                     round(self.segment.points[0][1],3))
                
                d = (round(self.segment.points[-1][0] + np.sin(self.segment.angle) * self.segment.link_width/2,3),
                     round(self.segment.points[-1][1],3))

                self.points = [(round(o[0] - np.sin(self.segment.angle) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3), o[1]),
                            (round(d[0] - np.sin(self.segment.angle) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3), d[1])]
            
            else:
                # horizontal link
                o = (round(self.segment.points[0][0],3),
                     round(self.segment.points[0][1] - np.cos(self.segment.angle) * self.segment.link_width/2,3))
                d = (round(self.segment.points[-1][0],3),
                     round(self.segment.points[-1][1] - np.cos(self.segment.angle) * self.segment.link_width/2,3))

                self.points = [(o[0], round(o[1] + np.cos(self.segment.angle) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3)),
                            (d[0], round(d[1] + np.cos(self.segment.angle) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3))]
        

            existing_coordinates = [node.coord for node in sim.nodes.values()]
            for point in self.points:
                if point not in existing_coordinates:
                    sim.create_node({'id': list(sim.nodes.keys())[-1] + 1, 'coord': tuple(point)})
                    existing_coordinates.append(point)
                    # print(self.segment.id, self.order, point, self.points, existing_coordinates)
            
            
            self.start_node = [k for k, v in sim.nodes.items() if v.coord == self.points[0]][0]
            self.end_node = [k for k, v in sim.nodes.items() if v.coord == self.points[-1]][0]
        
        elif self.segment.link_type == 'connector':
            positive_angle_start = np.rad2deg(self.segment.start_link.angle)
            positive_angle_end = np.rad2deg(self.segment.end_link.angle)

            if positive_angle_start < 0:
                positive_angle_start = 360 + positive_angle_start
            
            if positive_angle_end < 0:
                positive_angle_end = 360 + positive_angle_end

            
            if (positive_angle_start >= 45 and positive_angle_start <= 135) or (positive_angle_start >= 225 and positive_angle_start <= 315):
                # vertical start point
                o = (round(self.segment.points[0][0] + np.sin(positive_angle_start) * self.segment.link_width/2,3),
                     round(self.segment.points[0][1],3))
                
                d = (round(self.segment.points[-1][0],3),
                     round(self.segment.points[-1][1] - np.cos(positive_angle_end) * self.segment.link_width/2,3))
                
                self.points = [(round(o[0] - np.sin(positive_angle_start) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3), o[1]),
                               (d[0], round(d[1] + np.cos(positive_angle_end) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3))]
            
            else:
                # horizontal start point
                o = (round(self.segment.points[0][0],3),
                     round(self.segment.points[0][1] - np.cos(positive_angle_start) * self.segment.link_width/2,3))

                d = (round(self.segment.points[-1][0] + np.sin(positive_angle_end) * self.segment.link_width/2,3),
                     round(self.segment.points[-1][1],3))
                
                self.points = [(o[0], round(o[1] + np.cos(positive_angle_start) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3)),
                               (round(d[0] - np.sin(positive_angle_end) * (self.order * self.segment.lane_width + self.segment.lane_width/ 2),3), d[1])]
        

            existing_coordinates = [node.coord for node in sim.nodes.values()]
            for point in self.points:
                if point not in existing_coordinates:
                    sim.create_node({'id': list(sim.nodes.keys())[-1] + 1, 'coord': tuple(point)})
                    existing_coordinates.append(point)
                    # print(self.segment.id, self.order, point, self.points, existing_coordinates)
            
            
            self.start_node = [k for k, v in sim.nodes.items() if v.coord == self.points[0]][0]
            self.end_node = [k for k, v in sim.nodes.items() if v.coord == self.points[-1]][0]
        else: #connector
            self.points = (self.segment.points[0], self.segment.points[-1])
            self.start_node = self.segment.start_node
            self.end_node = self.segment.end_node

