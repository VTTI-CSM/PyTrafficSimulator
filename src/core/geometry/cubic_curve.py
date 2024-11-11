from .segment import Segment

CURVE_RESOLUTION = 50

class CubicCurve(Segment):
    def __init__(self, sim, config):
        # Set default configuration
        self.set_default_config_cubic()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties_cubic(sim, config)
    
    def set_default_config_cubic(self):  
        self.id = 0

        self.start_node = None
        self.start = (0,0)

        self.control_1 = (0, 0)
        self.control_2 = (0, 0)

        self.end_node = None
        self.end = (0,0)
        

        self.has_marking = False
        self.link_type = 'connector'
    
    def init_properties_cubic(self, sim, config={}):
        self.start = sim.nodes[self.start_node].coord
        self.end = sim.nodes[self.end_node].coord
        self.generate_path(sim, config)


    def generate_path(self, sim, config={}):
        # Generate path
        path = []
        for i in range(CURVE_RESOLUTION):
            t = i/CURVE_RESOLUTION
            x = t**3*self.end[0] + 3*t**2*(1-t)*self.control_2[0] + 3*(1-t)**2*t*self.control_1[0] + (1-t)**3*self.start[0]
            y = t**3*self.end[1] + 3*t**2*(1-t)*self.control_2[1] + 3*(1-t)**2*t*self.control_1[1] + (1-t)**3*self.start[1]
            path.append((x, y))
        
        config['points'] = path
        super().__init__(sim, config)
        
        # Arc-length parametrization
        normalized_path = self.find_normalized_path(CURVE_RESOLUTION)
        config['points'] = normalized_path
        super().__init__({sim, config})
       


    def compute_x(self, t):
        return t**3*self.end[0] + 3*t**2*(1-t)*self.control_2[0] + 3*(1-t)**2*t*self.control_1[0] + (1-t)**3*self.start[0]
    def compute_y(self, t):
        return t**3*self.end[1] + 3*t**2*(1-t)*self.control_2[1] + 3*(1-t)**2*t*self.control_1[1] + (1-t)**3*self.start[1]
    def compute_dx(self, t):
        return 3*t**2*(self.end[0]-3*self.control_2[0]+3*self.control_1[0]-self.start[0]) + 6*t*(self.control_2[0]-2*self.control_1[0]+self.start[0]) + 3*(self.control_1[0]-self.start[0])
    def compute_dy(self, t):
        return 3*t**2*(self.end[1]-3*self.control_2[1]+3*self.control_1[1]-self.start[1]) + 6*t*(self.control_2[1]-2*self.control_1[1]+self.start[1]) + 3*(self.control_1[1]-self.start[1])
