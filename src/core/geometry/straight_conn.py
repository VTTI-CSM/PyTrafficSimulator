from .segment import Segment
import numpy as np
from src.core.geometry.lane import Lane
from numpy import arctan2

class StrConnector(Segment):
    def __init__(self, sim, config):
        # Set default configuration
        self.set_default_config_str()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties_str(sim, config)  
    
    def set_default_config_str(self):  
        self.id = None

        self.start_link = None
        self.start_node = None
        self.start_coord = (0, 0)
        

        self.end_link = None
        self.end_node = None
        self.end_coord = (0, 0)

        self.points = [(0, 0), (0, 0)]

        self.order_start = None
        self.order_end = None

        self.number_of_lanes = None
        self.lane_width = 3.65
        self.lanes = []

        self.directions = []

        self.has_marking = False
        self.link_type = 'StrConn'
        self.length = None

    def init_properties_str(self, sim, config={}):
        if len(self.directions) == 0:
            for _ in range(self.number_of_lanes):
                self.directions.append(['THR'])

        if self.start_node == None: 
            self.start_node = self.start_link.end_node
            config['start_node'] = self.start_node

        if self.end_node == None: 
            self.end_node = self.end_link.start_node
            config['end_node'] = self.end_node

        self.start_coord = self.start_link.lanes[self.order_start].points[1]
        self.end_coord = self.end_link.lanes[self.order_end].points[0]

        if abs(np.rad2deg(self.start_link.angle)) == 90:
            #vertical
            self.start_coord = (self.start_link.lanes[self.order_start].points[1][0] + np.sin(self.start_link.angle) * (self.lane_width/2 - self.number_of_lanes/2 * self.lane_width),
                                 self.start_link.lanes[self.order_start].points[1][1])
            
            self.end_coord = (self.end_link.lanes[self.order_end].points[0][0] + np.sin(self.start_link.angle) * (self.lane_width/2 - self.number_of_lanes/2 * self.lane_width),
                               self.end_link.lanes[self.order_end].points[0][1])
            
            # print(self.start_link.lanes[self.order_start].points[1], self.start_coord)
            # print(self.end_link.lanes[self.order_end].points[0], self.end_coord)
        else:
            #horizontal
            self.start_coord = (self.start_link.lanes[self.order_start].points[1][0],
                                 self.start_link.lanes[self.order_start].points[1][1] - np.cos(self.start_link.angle) * (self.lane_width/2 - self.number_of_lanes/2 * self.lane_width))
            self.end_coord = (self.end_link.lanes[self.order_end].points[0][0],
                               self.end_link.lanes[self.order_end].points[0][1] - np.cos(self.start_link.angle) * (self.lane_width/2 - self.number_of_lanes/2 * self.lane_width))
            

            

        self.points = [self.start_coord, self.end_coord]

        self.angle = arctan2(self.points[1][1] - self.points[0][1], self.points[1][0] - self.points[0][0])
        self.link_width = self.lane_width * self.number_of_lanes


        lane_order = 0
        for _ in range(self.number_of_lanes):
            self.lanes.append(Lane(sim, {'id': lane_order, 'order': lane_order, 'segment':self, 'direction': self.directions[lane_order]}))
            lane_order += 1

        # self.start_coord = sim.nodes[self.start_node].coord
        # self.end_coord = sim.nodes[self.end_node].coord
        


        self.generate_path(sim, config)
    
    def generate_path(self, sim, config={}):
        self.points = [self.start_coord, self.end_coord]
        config['points'] = self.points
        config['directions'] = self.directions
        super().__init__(sim, config)

        self.has_marking = False