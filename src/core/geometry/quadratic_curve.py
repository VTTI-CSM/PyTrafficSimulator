from .segment import Segment
import numpy as np

PAS = 0.01
CURVE_RESOLUTION = 50

class QuadraticCurve(Segment):
    def __init__(self, sim, config):
        # Set default configuration
        self.set_default_config_quadratic()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties_quadratic(sim, config)  
    
    def set_default_config_quadratic(self):  
        self.id = 0

        self.start_link = None
        self.start_node = None
        self.start_coord = (0, 0)
        
        self.control = None

        self.end_link = None
        self.end_node = None
        self.end_coord = (0, 0)

        self.order_start = None
        self.order_end = None

        self.has_marking = False
        self.link_type = 'connector'

        self.number_of_lanes = 1

    def init_properties_quadratic(self, sim, config={}):
        if self.start_node == None: 
            self.start_node = self.start_link.end_node
            config['start_node'] = self.start_node

        if self.end_node == None: 
            self.end_node = self.end_link.start_node
            config['end_node'] = self.end_node
        
        self.start_coord = (self.start_link.lanes[self.order_start].points[1][0] + (self.number_of_lanes-1) * self.lane_width/2 * -np.sin(self.start_link.angle),
                             self.start_link.lanes[self.order_start].points[1][1] - (self.number_of_lanes-1) * self.lane_width/2 * -np.cos(self.start_link.angle))
        self.end_coord = (self.end_link.lanes[self.order_end].points[0][0] + (self.number_of_lanes-1) * self.lane_width/2 * -np.sin(self.end_link.angle),
                           self.end_link.lanes[self.order_end].points[0][1] - (self.number_of_lanes-1) * self.lane_width/2 * -np.cos(self.end_link.angle))
        # self.start_coord = sim.nodes[self.start_node].coord
        # self.end_coord = sim.nodes[self.end_node].coord

        if self.control == None: 
            if abs(np.rad2deg(self.start_link.angle)) == 90: self.control = (self.start_coord[0], self.end_coord[1])    #if the start lane is vertical
            else: self.control = (self.end_coord[0], self.start_coord[1])                                               #if the start lane is horizontal
            
            config['control'] = self.control

        self.generate_path(sim, config)
    
    def generate_path(self, sim, config={}):
        # Generate path
        path = []
        for i in range(CURVE_RESOLUTION):
            t = i/(CURVE_RESOLUTION-1)
            x = t**2*self.end_coord[0] + 2*t*(1-t)*self.control[0] + (1-t)**2*self.start_coord[0]
            y = t**2*self.end_coord[1] + 2*t*(1-t)*self.control[1] + (1-t)**2*self.start_coord[1]
            path.append((x, y))
        
        config['points'] = path
        super().__init__(sim, config)

        # Arc-length parametrization
        normalized_path = self.find_normalized_path(CURVE_RESOLUTION)
        config['points'] = normalized_path
        super().__init__(sim, config)


    def compute_x(self, t):
        return t**2*self.end_coord[0] + 2*t*(1-t)*self.control[0] + (1-t)**2*self.start_coord[0]
    def compute_y(self, t):
        return t**2*self.end_coord[1] + 2*t*(1-t)*self.control[1] + (1-t)**2*self.start_coord[1]
    def compute_dx(self, t):
        return 2*t*(self.end_coord[0]-2*self.control[0]+self.start_coord[0]) + 2*(self.control[0]-self.start_coord[0])
    def compute_dy(self, t):
        return 2*t*(self.end_coord[1]-2*self.control[1]+self.start_coord[1]) + 2*(self.control[1]-self.start_coord[1])