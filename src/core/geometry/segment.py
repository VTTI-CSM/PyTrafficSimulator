from scipy.spatial import distance
from scipy.interpolate import interp1d
from collections import deque
from numpy import arctan2, unwrap, linspace
from abc import ABC, abstractmethod
from math import sqrt
from scipy.integrate import quad
from src.core.geometry.lane import Lane

class Segment:

    instances = {}

    def __init__(self, sim, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties(sim)
        Segment.instances[self.id] = self

    def set_default_config(self):  
        self.id = 0
        self.start_node = None
        self.end_node = None

        # self.speed_limit = 16.7
        self.points = None
        self.vehicles = deque()
        self.lanes = []
        self.has_traffic_signal = False
        self.lane_width = 3.65
        self.number_of_lanes = 1
        self.has_marking = True

        self.u_f=40/3.6              # free flow speed (m/s) - 80kph
        self.u_c=30/3.6               # speed at capacity (m/s) - 60 kph
        self.q_c=1900.0             # capacity flow rate (veh/h) - unit is updated in init properties
        self.k_j=160.0              # jam density (veh/km) - unit is updated in init properties
        self.angle = None
        self.link_type = 'link'
        self.grade = 0

        self.directions = []
        self.detector = None
        



    def init_properties(self, sim):
        if self.points == None: self.points = [sim.nodes[self.start_node].coord, sim.nodes[self.end_node].coord]
        self.set_functions()
        
        if self.link_type == 'link': self.has_marking = True

        self.q_c=self.q_c/3600    # capacity flow rate (veh/s)
        self.k_j=self.k_j/1000     # jam density (veh/m)

        self.length = self.get_length()

        # Adding lanes to the segment
        # if self.link_type == 'link':
        lane_order = 0
        for _ in range(self.number_of_lanes):
            self.lanes.append(Lane(sim, {'id': lane_order, 'order': lane_order, 'segment':self, 'direction': self.directions[lane_order]}))
            lane_order += 1

                
    
    
    def set_traffic_signal(self, signal, phase_no):
        self.traffic_signal = signal
        self.traffic_signal_phase = phase_no
        self.has_traffic_signal = True


    @property
    def traffic_signal_state(self):
        if self.has_traffic_signal:
            i = self.traffic_signal_phase
            return self.traffic_signal.current_cycle[i]
        return True
    
    
    def set_functions(self):
        
        # Point
        self.get_point = interp1d(linspace(0, 1, len(self.points)), self.points, axis=0)
        self.angle = arctan2(self.points[1][1] - self.points[0][1], self.points[1][0] - self.points[0][0])
        self.link_width = self.lane_width * self.number_of_lanes

        if len(self.directions) == 0:
            for _ in range(self.number_of_lanes):
                self.directions.append(['THR', 'RT', 'LT'])

        # Heading
        headings = unwrap([arctan2(
            self.points[i+1][1] - self.points[i][1],
            self.points[i+1][0] - self.points[i][0]
        ) for i in range(len(self.points)-1)])
        if len(headings) == 1:
            self.get_heading = lambda x: headings[0]
        else:
            self.get_heading = interp1d(linspace(0, 1, len(self.points)-1), headings, axis=0)

    def get_length(self):
        length = 0
        for i in range(len(self.points) -1):
            length += distance.euclidean(self.points[i], self.points[i+1])
        return length

    def add_vehicle(self, veh):
        self.vehicles.append(veh)

    def remove_vehicle(self, veh):
        self.vehicles.remove(veh)

    @abstractmethod
    def compute_x(self, t):
        pass
    @abstractmethod
    def compute_y(self, t):
        pass
    @abstractmethod
    def compute_dx(self, t):
        pass
    @abstractmethod
    def compute_dy(self, t):
        pass

    def abs_f(self, t):
        return sqrt(self.compute_dx(t)**2 + self.compute_dy(t)**2)
    
    def find_t(self, a, L, epsilon):
        """  Finds the t value such that the length of the curve from a to t is L.

        Parameters
        ----------
        a : float
            starting point of the integral
        L : float
            target length
        epsilon : float
            precision of the approximation
        """
        
        def f(t):
            integral_value, _ = quad(self.abs_f, a, t)
            return integral_value
        
        # if we cannot reach the target length, return 1 
        if f(1) < L: return 1
        
        lower_bound = a
        upper_bound = 1
        mid_point = (lower_bound + upper_bound) / 2.0
        integ = f(mid_point)
        while abs(integ-L) > epsilon:
            if integ < L:       lower_bound = mid_point
            else:               upper_bound = mid_point
            mid_point = (lower_bound + upper_bound) / 2.0
            integ = f(mid_point)
        return mid_point
    
    def find_normalized_path(self, CURVE_RESOLUTION=50):
        normalized_path = [(self.compute_x(0), self.compute_y(0))]
        l = self.get_length()
        target_l = l/(CURVE_RESOLUTION-1)
        epsilon = 0.01
        a = 0
        for i in range(CURVE_RESOLUTION-1):
            t = self.find_t(a, target_l, epsilon)
            new_point = (self.compute_x(t), self.compute_y(t))
            normalized_path.append(new_point)
            if t == 1: break
            else:      a = t
        return normalized_path

# class Segment(Segment0):
#     def __init__(self, points):
#         super().__init__(points)

#     def compute_x(self, t):
#         # Implement x-coordinate calculation for your specific curve
#         pass

#     def compute_y(self, t):
#         # Implement y-coordinate calculation for your specific curve
#         pass

#     def compute_dx(self, t):
#         # Implement derivative of x-coordinate calculation for your specific curve
#         pass

#     def compute_dy(self, t):
#         # Implement derivative of y-coordinate calculation for your specific curve
#         pass
