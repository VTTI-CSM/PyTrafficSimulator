import numpy as np

class Detector:
    instances = []
    def __init__(self, segment, lane = None, config={}):
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties(segment, lane)

    def set_default_config(self):
        # same as integration
        self.id = 0
        self.type = 1           # type 1 detector is summarized at a link or section level, type 10 detector is provided on an individual lane basis, type 20 detector is provided on an individual lane basis for actuated signals.
        self.x = 5              # Detector station position measured as distance (m) from upstream end of link
        self.length = 2         # Effective detection length (m) of detector station [real]
        self.pulling_freq = 10  # Pulling frequency (seconds) for detector station
        self.name = ''          # Detector station name
        self.width = 2.5        # Width of the lane (m) where the detector is placed   (not in integration input file)
        self.cv_entry_t_list = []   # list of entry times of vehicles in the detector
        self.lane = None
        self.lane_order = None
        self.segment = None

        # self.detector_all = pd.DataFrame(columns=["Time_s", "DetectorID", "DetectorType", "SectionSpeed_kph", "SectionVolume_vph", "AverageLaneVolume_vph", "Occupancy", "ActivationFlag"]) #Type 1 detector
        # self.detector_all = pd.DataFrame(columns=["Time_s", "DetectorID", "DetectorType", "LaneNumber", "LaneVolume_vph", "LaneSpeed_kph", "Occupancy"]) #Type 10 detector
        
        self.vehicle_actuations = {}
        self.detector_call = False      # flag to indicate if the detector has been occupied by a vehicle
        

    
    def init_properties(self, segment, lane = None):
        self.segment = segment
        
        if lane:
            self.lane = lane
            self.lane_order = lane.order

        if lane:
            self.width = self.segment.lane_width - 1.5
        else:
            self.width = self.segment.link_width - 1.5

        self.get_coordinates()
        self.get_boundaries()
        
        
        
    
    def get_coordinates(self):
        if self.lane:
            if self.lane.points[0][0] == self.lane.points[-1][0]:
                if self.lane.points[0][1] > self.lane.points[-1][1]:
                    self.origin = (self.lane.points[0][0], self.lane.points[0][1] - self.x)
                else:
                    self.origin = (self.lane.points[0][0], self.lane.points[0][1] + self.x)
                self.coord1 = (self.origin[0] - self.width/2, self.origin[1] + self.length/2)
                self.coord2 = (self.origin[0] + self.width/2, self.origin[1] - self.length/2)
        
            else:
                if self.lane.points[0][0] > self.lane.points[-1][0]:
                    self.origin = (self.lane.points[0][0] - self.x, self.lane.points[0][1])
                else:
                    self.origin = (self.lane.points[0][0] + self.x, self.lane.points[0][1])
                self.coord1 = (self.origin[0] - self.length/2, self.origin[1] - self.width/2)
                self.coord2 = (self.origin[0] + self.length/2, self.origin[1] + self.width/2)
        
        else:
            if self.segment.points[0][0] == self.segment.points[-1][0]:
                if self.segment.points[0][1] > self.segment.points[-1][1]:
                    self.origin = (self.segment.points[0][0], self.segment.points[0][1] - self.x)
                else:
                    self.origin = (self.segment.points[0][0], self.segment.points[0][1] + self.x)
                self.coord1 = (self.origin[0] - self.width/2, self.origin[1] + self.length/2)
                self.coord2 = (self.origin[0] + self.width/2, self.origin[1] - self.length/2)
        
            else:
                if self.segment.points[0][0] > self.segment.points[-1][0]:
                    self.origin = (self.segment.points[0][0] - self.x, self.segment.points[0][1])
                else:
                    self.origin = (self.segment.points[0][0] + self.x, self.segment.points[0][1])
                self.coord1 = (self.origin[0] - self.length/2, self.origin[1] - self.width/2)
                self.coord2 = (self.origin[0] + self.length/2, self.origin[1] + self.width/2)
        return self.coord1, self.coord2
    
    def get_boundaries(self):
        self.detector_start = self.x - self.length/2
        self.detector_end = self.x + self.length/2
    
    def pull_reading_t_type1(self, curr_t, sim):
        t = self.pulling_freq * (curr_t//self.pulling_freq)
        volume_period = 0
        speed_period = []
        t_occupied_period = 0
        veh_id_prev = 0
        # columns=["Time_s", "DetectorID", "DetectorType", "SectionSpeed_kph", "SectionVolume_vph", "AverageLaneVolume_vph", "Occupancy", "ActivationFlag"]

        for veh_id in self.vehicle_actuations.keys():
            veh = sim.vehicles[veh_id]
            if self.vehicle_actuations[veh_id][0] >= t - self.pulling_freq and self.vehicle_actuations[veh_id][1] <= t:
                if self.vehicle_actuations[veh_id][1] > self.vehicle_actuations[veh_id][0]:
                    # vehicle counting
                    volume_period += 1
                
                    # vehicle speed - 3.6 is to convert m/s to km/h
                    speed_period.append(3.6 * (self.length + veh.l)/(self.vehicle_actuations[veh_id][1] - self.vehicle_actuations[veh_id][0]))

                    # vehicle occupancy
                    if veh_id_prev == 0: last_veh_exit_t = 0
                    else: last_veh_exit_t = self.vehicle_actuations[veh_id_prev][1]

                    t_occupied_period += self.vehicle_actuations[veh_id][1] - max(self.vehicle_actuations[veh_id][0], last_veh_exit_t)
            
            veh_id_prev = veh_id
        
        section_avg_vol = int(3600 * volume_period/self.pulling_freq)
        lane_avg_vol = section_avg_vol / self.segment.number_of_lanes
        section_occup = t_occupied_period/self.pulling_freq


        if len([veh_time for veh_time in self.vehicle_actuations.values() if veh_time[0] >= t - self.pulling_freq  and veh_time[1] <= t]) > 0 and len(speed_period) > 0:
            section_avg_spd = np.mean(speed_period)
            activation_flag = 1
        else:
            section_avg_spd = 0
            activation_flag = 0
        
        return [[t, self.id, self.type, section_avg_spd, section_avg_vol, lane_avg_vol, section_occup, activation_flag]]


    def pull_reading_t_type10(self, curr_t, lane_order, sim):
        t = self.pulling_freq * (curr_t//self.pulling_freq)
        volume_period = 0
        speed_period = []
        t_occupied_period = 0
        veh_id_prev = 0
        # ["Time_s", "DetectorID", "DetectorType", "LaneNumber", "LaneVolume_vph", "LaneSpeed_kph", "Occupancy"]

        for veh_id in self.vehicle_actuations.keys():
            veh = sim.vehicles[veh_id]
            if self.vehicle_actuations[veh_id][0] >= t and self.vehicle_actuations[veh_id][1] <= t + self.pulling_freq:
                if self.vehicle_actuations[veh_id][1] > self.vehicle_actuations[veh_id][0]:
                    if veh.current_lane.id == lane_order:
                        # vehicle counting
                        volume_period += 1
                    
                        # vehicle speed - 3.6 is to convert m/s to km/h
                        veh_speed = 3.6 * (self.length + veh.l)/(self.vehicle_actuations[veh_id][1] - self.vehicle_actuations[veh_id][0])
                        speed_period.append(veh_speed)

                        # vehicle occupancy
                        if veh_id_prev == 0: last_veh_exit_t = 0
                        else: last_veh_exit_t = self.vehicle_actuations[veh_id_prev][1]

                        t_occupied_period += self.vehicle_actuations[veh_id][1] - max(self.vehicle_actuations[veh_id][0], last_veh_exit_t)
            
            veh_id_prev = veh_id
        
        lane_avg_vol = int(3600 * volume_period/self.pulling_freq)

        section_occup = t_occupied_period/self.pulling_freq


        if len([veh_time for veh_time in self.vehicle_actuations.values() if veh_time[0] >= t  and veh_time[1] <= t + self.pulling_freq]) > 0 and len(speed_period) > 0:
            lane_avg_spd = np.mean(speed_period)
        else:
            lane_avg_spd = 0
        
        return [[t, self.id, self.type, lane_order, lane_avg_vol, lane_avg_spd, section_occup]]



    def pull_data(self, sim):
        #generating the whole detector file
        if self.type == 1: self.detector_all = np.empty((0, 8))
        elif self.type == 10: self.detector_all = np.empty((0, 7))
        
        if sim.t > self.pulling_freq:
            for t in range(int(self.pulling_freq), int(sim.t), int(self.pulling_freq)):
                if self.type == 1:
                    self.detector_all = np.concatenate((self.detector_all, self.pull_reading_t_type1(t, sim)))

                elif self.type == 10:
                    for lane_order in range(0, self.segment.number_of_lanes):
                        self.detector_all = np.concatenate((self.detector_all, self.pull_reading_t_type10(t, lane_order, sim)))
    
        return self.detector_all
    
    def update(self, sim):
        if self.type == 20: veh_list = self.lane.vehicles
        else: veh_list = self.segment.vehicles
        self.detector_call = False

        for veh in veh_list:
            #detecting when a vehicle enters the loop area
            if veh.x > self.detector_start and veh.x_prev <= self.detector_start:
                self.vehicle_actuations[veh.id] = [sim.t, sim.t]
                if veh.veh_class == 2 and veh.ECO_CACC: self.cv_entry_t_list.append(sim.t)
            
            #detecting when a vehicle exits the loop area
            if veh.x >= self.detector_end + veh.l and veh.x_prev < self.detector_end + veh.l:
                self.vehicle_actuations[veh.id][1] = sim.t
            
            if veh.x >= self.detector_start and veh.x <= self.detector_end + veh.l: self.detector_call = True