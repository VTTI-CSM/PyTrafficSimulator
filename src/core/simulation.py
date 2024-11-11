from .vehicle_generator import VehicleGenerator
from .geometry.quadratic_curve import QuadraticCurve
from .geometry.cubic_curve import CubicCurve
from .geometry.straight_conn import StrConnector
from .geometry.segment import Segment
from .geometry.node import Node
from .infrastructure.detector import Detector
from .vehicle import Vehicle
from .infrastructure.traffic_signal import TrafficSignal
from .infrastructure.traffic_signal_nema import TrafficSignalNEMA
from .geometry.shortestpath import Dijkstra
import pandas as pd
import numpy as np
import os
import pickle
from matplotlib import pyplot as plt
from sklearn.metrics import r2_score, mean_absolute_error, mean_absolute_percentage_error
directory = os.path.abspath('')
directory = directory[slice(0,directory.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

class Simulation:
    def __init__(self, duration, random_seed, resolution=1/10, destroy=False, FR_param = [0.74375, 1.0, 0.1]):
        self.duration = duration
        self.rseed = random_seed
        self.nodes = {}
        self.detectors = {}
        self.segments = []
        self.vehicles = {}
        self.vehicle_generator = []

        self.t = 0.0
        self.frame_count = 0
        self.dt = resolution
        self.traffic_signals = []
        self.current_vehicle_count = []
        self.current_queue_count = []
        self.last_time_vehicle_is_counted = 0
        self.stochastic = False
        self.deterministic = False
        self.CV_queue_Est = False
        self.MP = 0
        self.destroy = destroy

        #driver behavior parameters - HDV
        self.FR_param_a = FR_param[0]
        self.FR_param_b = FR_param[1]
        self.FR_param_d = FR_param[2]

        #driver behavior parameters - AV
        self.FR_param_a_AV = 0.1
        self.FR_param_b_AV = 10.0
        self.FR_param_d_AV = 0.1
        

    
    def get_total_vehicle_count(self, specific_veh_class = None):
        # return Vehicle.count
        count = len(self.vehicles)
        if specific_veh_class:
            count = 0
            for veh in self.vehicles.values():
                if veh.veh_class == specific_veh_class: count += 1
        return count
    
    def get_current_vehicle_count(self):
        # return Vehicle.count
        count = 0
        for seg in self.segments:
            for lane in seg.lanes:
                count += len(lane.vehicles)
        return [int(self.t), count]

    def get_current_queue_count(self):
        # columns=['Time', 'EBT' , 'EBL', 'WBT', 'WBL', 'SBT', 'SBL', 'NBT', 'NBL']
        count = [int(self.t), 0, 0, 0, 0, 0, 0, 0, 0]
        
        for seg in self.segments:
            if seg.has_traffic_signal:
                angle = round(np.degrees(seg.angle),0)
                if angle == 0: i_th, i_lt = 1, 2
                elif angle == 180: i_th, i_lt = 3, 4
                elif angle == 90: i_th, i_lt = 5, 6
                else: i_th, i_lt = 7, 8

                count_Thr = count[i_th]
                count_LT = count[i_lt]
                for lane in seg.lanes:
                    if 'THR' in lane.direction:
                        count_Thr += len([veh.id for veh in lane.vehicles if veh.v < 1.25])
                    else:
                        count_LT += len([veh.id for veh in lane.vehicles if veh.v < 1.25])
                count[i_th] = count_Thr
                count[i_lt] = count_LT
        return count

    def get_average_stops(self, specific_veh_class = None):
        # return average stops
        total_stops = 0
        if specific_veh_class:
            for veh in self.vehicles.values():
                if veh.veh_class == specific_veh_class: total_stops += veh.stops
        else:
            for veh in self.vehicles.values():
                total_stops += veh.stops
        average_stops = total_stops / self.get_total_vehicle_count(specific_veh_class)
        return average_stops
    
    def get_average_fuel(self, specific_veh_class = None):
        # return average fuel consumption
        total_fuel = 0
        if specific_veh_class:
            for veh in self.vehicles.values():
                if veh.veh_class == specific_veh_class: total_fuel += veh.fc
        else:
            for veh in self.vehicles.values():
                total_fuel += veh.fc
        average_fuel = total_fuel / self.get_total_vehicle_count(specific_veh_class)
        return average_fuel * 1000
    
    def generate_trajectory_file(self, specific_veh_class = None):
        # return trajectory file for all vehicles
        columns = ["Time_s", "VehID", "VehClass", "VehType", "CumDistTravOrigin_m", "LinkNo", "LaneNo", "CumDistTravLink_m", "Speed_kph", "Spacing_m"]
        self.traj_df = pd.DataFrame(columns=columns)
        if specific_veh_class:
            
            # for veh in Vehicle.instances:
            for veh in self.vehicles.values():
                if veh.veh_class == specific_veh_class: 
                    self.traj_df = pd.concat([self.traj_df, pd.DataFrame(veh.traj_array_veh, columns=columns)], ignore_index=True)
        else:
            for veh in self.vehicles.values():
                self.traj_df = pd.concat([self.traj_df, pd.DataFrame(veh.traj_array_veh, columns=columns)], ignore_index=True)


        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        print(self.traj_df)
        self.traj_df.to_pickle(dir + r'\output\Trajectory.pkl')
    
    def generate_detector_file(self):
        if list(self.detectors.values())[0].type == 1: columns = ["Time_s", "DetectorID", "DetectorType", "SectionSpeed_kph", "SectionVolume_vph", "AverageLaneVolume_vph", "Occupancy", "ActivationFlag"]
        elif list(self.detectors.values())[0].type == 10: columns = ["Time_s", "DetectorID", "DetectorType", "LaneNumber", "LaneVolume_vph", "LaneSpeed_kph", "Occupancy"]

        self.detector_df = pd.DataFrame(columns=columns)

        for detector in self.detectors.values():
            self.detector_df = pd.concat([self.detector_df, pd.DataFrame(detector.pull_data(self), columns=columns)], ignore_index=True)
        
        print(self.detector_df)
        
        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        self.detector_df.to_csv(dir + r'\output\detector.csv', index=False)
    
    def generate_signal_indication_file(self, file_name='signal.csv'):
        columns = ["Time", "SignalID", "Phase", "SignalState"]
        self.signal_df = pd.DataFrame(columns=columns)

        for signal in self.traffic_signals:
            self.signal_df = pd.concat([self.signal_df, pd.DataFrame(signal.signal_indication_array, columns=columns)], ignore_index=True)
        
        # print(self.signal_df)
        print("Signal Indication File Generated")
        
        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        self.signal_df.to_csv(dir + r'\output\\' + str(file_name), index=False)

    def generate_vehicle_no_file(self, file_name='number_of_veh.csv'):
        self.vehno_df = pd.DataFrame(self.current_vehicle_count, columns=['Time', 'NoOfVeh'])

        # print(self.vehno_df)
        print("Vehicle Number File Generated")
        
        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        self.vehno_df.to_csv(dir + r'\output\\' + str(file_name), index=False)
    
    def generate_delay_file(self, file_name='avg_delay.csv'):
        self.vehno_df = pd.DataFrame(self.current_avg_delay, columns=['Time', 'avg_delay'])


        print("Average Delay File Generated")
        
        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        self.vehno_df.to_csv(dir + r'\output\\' + str(file_name), index=False)

    
        

    def generate_queue_file(self, file_name = 'queues.csv'):
        self.queue_df = pd.DataFrame(self.current_queue_count, columns=['Time', 'EBT' , 'EBL', 'WBT', 'WBL', 'SBT', 'SBL', 'NBT', 'NBL'])

        # print(self.queue_df)
        print("Queue File Generated")
        
        #save to csv
        dir = os.path.abspath('')
        dir = dir[slice(0,dir.find('PyTrafficSimulator_CSM') + len('PyTrafficSimulator_CSM'),1)]

        self.queue_df.to_csv(dir + r'\output\\' + str(file_name), index=False)

    def add_vehicle(self, veh):
        self.vehicles[veh.id] = veh
        veh.choose_lane('self')

        if len(veh.path) > 0:
            for seg in self.segments:
                if seg.id == veh.path[0]: seg.add_vehicle(veh)
            # self.segments[veh.path[0]].add_vehicle(veh)

    def add_segment(self, seg):
        self.segments.append(seg)
    
    def add_node(self, node):
        self.nodes[node.id] = node

    def add_detector(self, detector):
        self.detectors[detector.id] = detector

    def add_vehicle_generator(self, gen):
        self.vehicle_generator.append(gen)
        if gen.Stochastic: self.stochastic = True
        if gen.Deterministic: self.deterministic = True 

    
    def create_vehicle(self, **kwargs):
        veh = Vehicle(self, kwargs)
        self.add_vehicle(veh)

    
    def create_segment(self, config={}):
        seg = Segment(self, config)
        self.add_segment(seg)
        return seg
    
    def create_node(self, config={}):
        node = Node(config)
        self.add_node(node)
    
    def create_straight_conn(self, config={}):
        conn = StrConnector(self, config)
        self.add_segment(conn)
        return conn

    def create_quadratic_bezier_curve(self, config={}):
        cur = QuadraticCurve(self, config)
        self.add_segment(cur)
        return cur

    def create_cubic_bezier_curve(self, start, control_1, control_2, end):
        cur = CubicCurve(start, control_1, control_2, end)
        self.add_segment(cur)
        return cur

    def create_vehicle_generator(self, **kwargs):
        gen = VehicleGenerator(kwargs)
        self.add_vehicle_generator(gen)

    def create_signal(self, lanes, config={}):
        sig = TrafficSignal(self, lanes, config)
        self.traffic_signals.append(sig)
        return sig
    
    def create_signal_nema(self, config={}):
        sig = TrafficSignalNEMA(self, config)
        self.traffic_signals.append(sig)
        return sig
    
    def create_detector(self, seg, config={}):
        for this_seg in self.segments:
            if this_seg.id == seg: 
                segment = this_seg

        if config['type'] != 20: 
            det = Detector(segment, config = config)
            self.add_detector(det)
            segment.detector = det
        elif config['type'] == 20:
            for lane in segment.lanes:
                config['id'] = config['id'] + lane.order
                det = Detector(segment, lane, config=config)
                self.add_detector(det)
                lane.detector = det

        return det


    def run(self, steps):
        for _ in range(steps):
            self.update()
    
    def initialize(self):
        pass

    def update(self):
        for signal in self.traffic_signals:
            if signal.NB_Activation: 
                signal.NB_obj.update(self)
            elif signal.actuated: 
                signal.update_actuated(self)
            else:
                signal.update_fixed(self)

            
            # signal.update(self)
            # print(signal.current_phase_index, signal.get_time_to_switch(self, phase_index=1))
        
        # Update vehicles
        for segment in self.segments:
            for lane in segment.lanes:
                # updating time to switch for each link in segment.
                if segment.has_traffic_signal:
                    if self.deterministic: lane.ts_det = round(segment.traffic_signal.get_time_to_switch_det(self.t, phase_index=lane.traffic_signal_phase),1) + 0.1
                    if self.stochastic: lane.ts_stoch = round(segment.traffic_signal.get_time_to_switch_stoch(self.t,phase_index=lane.traffic_signal_phase),1) + 0.1
                
                # this accounts for start loss
                
                if segment.has_traffic_signal and lane.traffic_signal_state == 'red':
                    # if segment.id == 4: 
                        # print(lane.traffic_signal_state, lane.last_red_indication_t)
                    # print(segment.id, lane.traffic_signal_state, lane.last_red_indication_t, self.t)
                    lane.last_red_indication_t =  self.t

                    


                # signal_state = lane.traffic_signal_state     #True if green
                
                



                for i in range(0, len(lane.vehicles)):
                    stop = 0
                    if len(lane.vehicles) > 1 and i > 0:
                        lead_vehicle = lane.vehicles[i-1]
                        lead_spacing = lead_vehicle.x - lane.vehicles[i].x


                        # if lead_vehicle.v < lane.vehicles[i].v: 
                        #     time_to_collision = lead_spacing / (lane.vehicles[i].v - lead_vehicle.v)
                        # else: time_to_collision = 99999
                        # specifying look ahead distance
                        # if lead_spacing > (1/segment.k_j) + 1.2 * 0.95 * segment.u_f:
                        # if time_to_collision > 5:

                        if lane.vehicles[i].veh_class == 1: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f * 1.0
                        else: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f * 1.0
                        # if lane.vehicles[i].veh_class == 1: look_ahead_distance = (1/segment.k_j) + 1.2 * lane.vehicles[i].v * 1.0
                        # else: look_ahead_distance = (1/segment.k_j) + 1.2 * lane.vehicles[i].v * 1.0

                        # if lane.vehicles[i].veh_class == 1: look_ahead_distance = (1/segment.k_j) + (lane.vehicles[i].v**2)/ (6)
                        # else: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f


                        if lead_spacing > look_ahead_distance:
                            lead_vehicle = None
                            lead_spacing = 99999

                        
                    
                    else:
                        # when there is no vehicles ahead in that link
                        current_segment_index = lane.vehicles[i].path.index(segment.id)
                        if current_segment_index + 1 == len(lane.vehicles[i].path):
                            next_segment = None
                        else:
                            next_segment_id = lane.vehicles[i].path[lane.vehicles[i].path.index(segment.id)+1]
                            next_segment = next((seg for seg in self.segments if seg.id == next_segment_id), None)
                        if next_segment:
                            next_lane = lane.vehicles[i].which_potential_lane(next_segment)
                            # print(next_lane.segment.id, next_lane.order)
                            if len(next_lane.vehicles) > 0:
                                lead_vehicle = next_lane.vehicles[-1]
                                lead_spacing = lane.vehicles[i].current_seg.length - lane.vehicles[i].x + lead_vehicle.x


                                # if lead_vehicle.v < lane.vehicles[i].v: 
                                #     time_to_collision = lead_spacing / (lane.vehicles[i].v - lead_vehicle.v)
                                # else: time_to_collision = 99999
                                # if lane.vehicles[i].id == 15: 
                                #     print(time_to_collision)
                                # if time_to_collision > 5:
                                
                                if lane.vehicles[i].veh_class == 1: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f * 1.5
                                else: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f * 1.5
                                # if lane.vehicles[i].veh_class == 1: look_ahead_distance = lane.vehicles[i].look_ahead_distance
                                # else: look_ahead_distance = (1/segment.k_j) + 1.5 * segment.u_f * 1.5

                                if lead_spacing > look_ahead_distance:
                                    lead_vehicle = None
                                    lead_spacing = 99999
                            else: 
                                lead_vehicle = None
                                lead_spacing = 99999
                        else: 
                            lead_vehicle = None
                            lead_spacing = 99999
                        
                    lane.vehicles[i].update(lead_vehicle, lead_spacing, self.dt, self.t)

                    # update front vehicle detection data. Class 2 is connected vehicle class
                    if lane.vehicles[i].veh_class == 2 and i > 0:
                        lane.vehicles[i].leader = lane.vehicles[i-1]
                    else: lane.vehicles[i].leader = None
                    # if stop == 1: 
                    #     print(self.t, lane.vehicles[i].x, lane.vehicles[i].a)
            


        # Check roads for out of bounds vehicle
        for segment in self.segments:
            for lane in segment.lanes:
                # If road has no vehicles, continue
                if len(lane.vehicles) == 0: continue
                # If not
                else:
                    vehicle_list = list(lane.vehicles)
                    for vehicle in vehicle_list:
                        # If first vehicle is out of road bounds
                        if vehicle.x >= segment.length:
                            diff = vehicle.x - segment.length
                            # If vehicle has a next road
                            if vehicle.current_road_index + 1 < len(vehicle.path):
                                # Update current road to next road
                                vehicle.current_road_index += 1
                                # Add it to the next road
                                next_road_index = vehicle.path[vehicle.current_road_index]
                                #adding vehicle to the segment and assigning a lane
                                segment.remove_vehicle(vehicle)

                                for seg in self.segments:
                                    if seg.id == next_road_index: 
                                        seg.add_vehicle(vehicle)
                                        break

                                vehicle.current_lane.remove_vehicle(vehicle)
                                vehicle.choose_lane(seg)
                                # vehicle.choose_lane(self.segments[next_road_index])
                            else:
                                # In all other cases, remove it from its road
                                segment.remove_vehicle(vehicle)
                                vehicle.current_lane.remove_vehicle(vehicle)

                            # Reset vehicle properties
                            vehicle.x = diff
                            # vehicle.x = 0

        
        
        # Update vehicle generators
        for gen in self.vehicle_generator:
            gen.update(self)
        
        #update detectors
        for det in self.detectors.values():
            det.update(self)
        
        
        
        if int(self.t) != self.last_time_vehicle_is_counted: 
            self.current_vehicle_count.append(self.get_current_vehicle_count())
            self.current_queue_count.append(self.get_current_queue_count())

        self.last_time_vehicle_is_counted = int(self.t)


        # considers CVs in queue estimation
        for gen in self.vehicle_generator:
            for veh in gen.vehicles:
                if veh[1]['veh_class'] == 2 and veh[0] > 0:
                    self.CV_queue_Est = True
                    break
        
        # Increment time
        self.t += self.dt
        self.frame_count += 1
