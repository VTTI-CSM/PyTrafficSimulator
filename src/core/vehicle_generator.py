from .vehicle import Vehicle
from numpy import random
import numpy as np
from .geometry.shortestpath import Dijkstra
import math
import scipy.stats as stats
from collections import deque
from scipy.optimize import brentq

class VehicleGenerator:
    # last_added_time = 0

    def __init__(self, sim, config={}):
        # Set default configurations
        self.set_default_config(sim)

        # Update configurations
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties(sim, config)

    def set_default_config(self, sim):
        """Set default configuration"""
        self.sim = sim
        self.random = random
        self.random.seed(self.sim.rseed)
        self.flow_rate = 0
        # self.interarrival_time = 36         #first vehicle input time.
        self.interarrival_time = 0         #first vehicle input time.
        self.vehicles = [(1, {})]
        self.last_added_time = 0
        self.end_time = 3600
        self.start_time = 0
        self.arrival_type = 'poisson' # 'uniform' or 'poisson'
        self.Stochastic = False
        self.Deterministic = False
        self.time_diff_queue = deque()
        self.time_diff = 0
        # VehicleGenerator.last_added_time = 0

    def init_properties(self, sim, config={}):
        # self.interarrival_time = self.start_time
        # calculate path
        dijk_obj = Dijkstra(sim)

        for i in range(len(self.vehicles)):
            try: self.Stochastic = config['vehicles'][i][1]['ECO_CACC_STOCH']
            except: pass

            try: self.Deterministic = config['vehicles'][i][1]['ECO_CACC_DET']
            except: pass
        
        for (weight, config) in self.vehicles:
            config['path'] = dijk_obj.get_path(config['start_node'], config['end_node'])
            # print(config['path'])


        self.upcoming_vehicle = self.generate_vehicle()

        self.flow_rate = 0
        for (weight, config) in self.vehicles:
            self.flow_rate += weight
        
        if self.flow_rate <= 1: 
            self.interarrival_time = 0
        else:
            self.interarrival_time = round(3600 / self.flow_rate,1)

    def generate_vehicle(self):
        """Returns a random vehicle from self.vehicles with random proportions"""
        total = sum(pair[0] for pair in self.vehicles)
        if self.flow_rate > 0:
            r = self.random.randint(1, total+1)
            for (weight, config) in self.vehicles:
                r -= weight
                if r <= 0:
                    return Vehicle(self.sim, config)



    def update(self, simulation):
        """Add vehicles"""      
        # if simulation.t - VehicleGenerator.last_added_time >= 60 / self.vehicle_rate:


        # Generate vehicles randomly using exponential interarrival times (poisson-distributed arrilvals)
        if self.flow_rate > 0:
            if simulation.t - self.last_added_time >= self.interarrival_time and simulation.t < self.end_time and simulation.t >= self.start_time:
                # Generate a new vehicle to be readu for addition
                self.upcoming_vehicle = self.generate_vehicle()

        
                # If time elasped after last added vehicle is
                # greater than vehicle_period; generate a vehicle
                segment = simulation.segments[self.upcoming_vehicle.path[0] - 1]
                potential_lane = self.upcoming_vehicle.which_potential_lane(segment)

                if len(potential_lane.vehicles) == 0 or potential_lane.vehicles[-1].x >= self.upcoming_vehicle.s0:
                    
                    # If there is space for the generated vehicle; add it
                    simulation.add_vehicle(self.upcoming_vehicle)
                    
                    # Reset last_added_time and upcoming_vehicle
                    if simulation.t > 110:
                        stop = 1
                    self.last_added_time = simulation.t

                    Vehicle.count += 1  # increment count by one for each new instance
                    # print('adding vehicle at ', VehicleGenerator.last_added_time)


                
                    # elif simulation.vehicles[segment.vehicles[-1]].x <= self.upcoming_vehicle.s0 + self.upcoming_vehicle.l:
                    #TODO: If there is no space for the generated vehicle, add to deffered vehicles count
                    
                    
                    #next arrival time
                    if self.arrival_type == 'poisson':
                        # var_perc = 0.2*3    #for Eco-CACC work
                        var_perc = 1.0      #for NB work

                        # scale = 3600/self.flow_rate
                        # lower, upper = 3600/1800, 999999
                        # self.interarrival_time = (1 - var_perc) * 2 + var_perc * self.random.exponential(scale=3600 / self.flow_rate, size=1)[0]

                        
                        X = max(var_perc*2, math.floor(self.random.exponential(scale=var_perc * 3600 / self.flow_rate, size=1)[0]*10)/10)
                        # print(X)
                        if X < var_perc * 2:
                            self.time_diff += var_perc * 2 - X
                            X = var_perc * 2
                        else:
                            if self.time_diff > 0 and X > var_perc * 2:
                                if X - self.time_diff >= var_perc * 2.0:
                                    X = X - self.time_diff
                                    self.time_diff = 0
                                else:
                                    self.time_diff = self.time_diff - (X - var_perc * 2)
                                    X = var_perc * 2
                        self.interarrival_time = (1 - var_perc) * 3600/self.flow_rate + X


                        

                    elif self.arrival_type == 'uniform':
                        self.interarrival_time = round(3600 / self.flow_rate,1)
            
