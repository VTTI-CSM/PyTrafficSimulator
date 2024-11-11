import numpy as np
import random
from src.core.models.fr_model import FRModel
from src.core.models.rpa_model import RPAModel
from src.core.models.fc_model import FCModel
from src.core.geometry.segment import Segment
from src.core.models.dynamics import Dynamics

class Vehicle:
    count = 0 # initialize count to zero
    def __init__(self, sim, config={}):
        # print("adding_vehicle")
        # Set default configuration
        
        self.set_default_config(sim)

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)

        # Calculate properties
        self.init_properties(config)
        
        
    def set_default_config(self, sim):
        self.sim = sim
        # self.id = uuid.uuid4() - remove
        self.id = len(sim.vehicles) + 1
        self.l = 4.88
        self.s0 = 6.25      #initial spacing  - Minimum spacing at capacity
        self.T = 1
        self.v_max = None
        self.delay = 0
        self.stops = 0
        self.fc = 0
        self.v_prev = 0 # speed at the prevoius time step
        self.x_prev = 0 # location at the prevoius time step
        self.a_prev = 0 # acceleration at the prevoius time step

        self.path = []
        self.veh_class = 1
        self.veh_type = 1        

        self.x = 0
        self.x_cumm = 0
        self.current_road_index = None
        self.current_seg = None         #link object
        self.current_lane = None
        self.current_spacing = 0

        self.v = 0              # m/s
        self.v_0 = 0
        self.a = 0              # m/s2
        self.stopped = False

        self.next_turn = None       #either 'THR', 'LT', OR'RT' 

        self.ECO_CACC_DET = False
        self.ECO_CACC_STOCH = False
        self.ECO_CACC = None
        self.Eco_CACC_consider_queue = False
        
        self.ECO_CACC_US_cont_dist = 370    #meters
        self.ECO_CACC_DS_cont_dist = 200    #meters
        self.x_last_stopbar = -1

        # self.traj_df_veh = pd.DataFrame(columns=["Time_s", "VehID", "VehClass", "VehType", "CumDistTravOrigin_m", "LinkNo", "LaneNo", "CumDistTravLink_m", "Speed_kph", "Spacing_m"])
        self.traj_array_veh = np.empty((0, 10)).tolist()

        self.leader = None      #only taakes a value when the vehicle is a connected vehicle and has sensors to detect the vehicle ahead.
        self.tq_prev = 0
        self.qbackx_prev = 99999
        self.skip_us_control = False
        self.look_ahead_distance = 6.25

    def init_properties(self, config):
        random.seed(self.sim.rseed)
        self.current_seg = Segment.instances[self.path[0]]
        if self.v_0 == 0: self.v_0 = self.current_seg.u_f - 0.001
        self.s0 = 1 / self.current_seg.k_j
        self.v = self.v_0
        self.v_prev = self.v_0
        self.current_road_index = 0
        self.next_turn = self.get_next_turn()

        if self.ECO_CACC_DET or self.ECO_CACC_STOCH:
            self.ECO_CACC = ECO_ACC

        
        
        if self.ECO_CACC:
            # automated driving parameters
            # FRModel.a, FRModel.b, FRModel.d = 0.1, 10, 0.1
            FRModel.a, FRModel.b, FRModel.d = self.sim.FR_param_a_AV, self.sim.FR_param_b_AV, self.sim.FR_param_d_AV 
        else:
            # driver behavior parameters (HDV)
            # FRModel.a, FRModel.b, FRModel.d = 0.74375, 1.0, 0.1  
            FRModel.a, FRModel.b, FRModel.d = self.sim.FR_param_a, self.sim.FR_param_b, self.sim.FR_param_d

    def get_next_turn(self):
        link1 = self.current_seg
        try: link2 = Segment.instances[self.path[self.path.index(self.current_seg.id) + 2]]
        except: link2 = self.current_seg

        if link1.id == link2.id: result = 'THR'
        else:
            # get angle of link 2 from link 1
            angle1 = np.rad2deg(link1.angle)
            angle2 = np.rad2deg(link2.angle)

            if angle2 - angle1 == 90 or angle2 - angle1 == -270: result = 'RT'
            elif angle2 - angle1 == -90 or angle2 - angle1 == 270: result = 'LT'
            else: result = 'THR'

            return result

    
    def choose_lane(self, segment):
        # if self.current_seg.id == 0:
        #     stop = 1
        if segment == 'self':  segment = self.current_seg


        if self.current_lane == None:
            entry_headway = {}
            for lane_ in segment.lanes:
                if self.next_turn in lane_.direction:
                    if len(lane_.vehicles) == 0: entry_headway[lane_] = 99999
                    else: entry_headway[lane_] = lane_.vehicles[-1].x
                else: continue
            
            max_value = max(entry_headway.values())
            max_keys = [key for key, value in entry_headway.items() if value == max_value]  # Collect keys with maximum value
            chosen_lane = random.choice(max_keys)
        else:
            chosen_lane = None
            min_val = 99999
            for lane in segment.lanes:
                dist_between_lanes = np.sqrt((lane.points[0][0] - self.current_lane.points[-1][0])**2 + (lane.points[0][1] - self.current_lane.points[-1][1])**2)
                if dist_between_lanes < min_val:
                    min_val = dist_between_lanes
                    chosen_lane = lane

        #Assign this vehicle to the chosen lane
        chosen_lane.add_vehicle(self)
        self.current_lane = chosen_lane
        self.current_seg = segment    


    def which_potential_lane(self, segment):
        if self.current_lane == None:
            entry_headway = {}
            for lane_ in segment.lanes:
                if self.next_turn in lane_.direction:
                    if len(lane_.vehicles) == 0: entry_headway[lane_] = 99999
                    else: entry_headway[lane_] = lane_.vehicles[-1].x
                else: continue
            
            max_value = max(entry_headway.values())
            max_keys = [key for key, value in entry_headway.items() if value == max_value]  # Collect keys with maximum value
            chosen_lane = random.choice(max_keys)
        else:
            chosen_lane = None
            min_val = 99999
            for lane in segment.lanes:
                dist_between_lanes = np.sqrt((lane.points[0][0] - self.current_lane.points[-1][0])**2 + (lane.points[0][1] - self.current_lane.points[-1][1])**2)
                if dist_between_lanes < min_val:
                    min_val = dist_between_lanes
                    chosen_lane = lane

        return chosen_lane

    

    def update(self, lead, lead_spacing, dt, current_time):
        # Update position and velocity
        # Fadhlon-Rakha Car-Following model        
        # check if the vehicle is going to stop in the next time step
        self.v_prev = self.v
        self.x_prev = self.x
        self.a_prev = self.a

        if self.v + self.a*dt < 0 and self.a != 0:
            dx = 1/2*self.v*self.v/self.a
            self.x -= dx
            self.x_cumm -= dx
            self.v = 0
            
        else:
            self.v += self.a*dt
            # self.v = int(self.v*1000)/1000
            if self.v < 0.1 and self.a <= 0: self.v = 0

            dx = self.v*dt + self.a*dt*dt/2
            self.x += dx
            self.x_cumm += dx

        current_segment = self.current_seg

        # Update delay
        self.delay += max(0,dt * (1 - (self.v / current_segment.u_f)))
        # Update number of Stops
        if self.v < self.v_prev: self.stops += (self.v_prev - self.v) / max(self.v, current_segment.u_f)
        # Update fuel consumption
        self.fc += FCModel.fc_main(self.a, (self.v_prev + self.v)/2, current_segment.grade) * dt



        

        # update distance to intersection 
        if self.current_lane.has_traffic_signal: 
            DTI = max(0, current_segment.length - self.x + 1/current_segment.k_j - 0.5 - self.l)
            delta_x_signal = DTI + self.l   # this is a virtual spacing for stop bar, used only as input to car following model.
        else: 
            DTI = 0
            delta_x_signal = 0

        if self.id == 14:
            stop = 1
            # print(self.sim.t, delta_x_signal)

        if self.x_last_stopbar == -1 and DTI == 0: 
            self.x_last_stopbar = self.x_cumm


        
        a_max = Dynamics.get_max_acceleration(self.v, current_segment.grade)
        v_at_stopbar = min(np.sqrt(max(0, self.v**2 + 2 * DTI * a_max)), current_segment.u_f)
        time_to_cross = (DTI) * 2 / max(0.01, (v_at_stopbar + self.v - 0.1))
        
        # Accounting for signal state at arrival
        curr_signal_state = self.current_lane.traffic_signal_state
        if self.ECO_CACC and curr_signal_state != 'red' and self.v > 2:
            # arrival_t = self.sim.t + (2*DTI/(self.v + current_segment.u_f))
            arrival_t = self.sim.t + time_to_cross
            signal_state = self.current_lane.future_traffic_signal_state(arrival_t)
        else:
            arrival_t = self.sim.t
            signal_state = self.current_lane.traffic_signal_state
        
        # checking queue presence
        queue_present = False
        # if self.ECO_CACC and self.Eco_CACC_consider_queue and self.current_lane.has_traffic_signal and self.x < self.qbackx_prev and self.leader and int(self.leader.v) == 0:
        # if self.ECO_CACC and self.Eco_CACC_consider_queue and self.current_lane.has_traffic_signal and self.x < self.qbackx_prev and self.v > 1.25 and self.qbackx_prev < 250:
        if self.ECO_CACC and self.Eco_CACC_consider_queue and self.current_lane.has_traffic_signal and self.x < self.qbackx_prev and self.qbackx_prev < 250 and self.v > 1.25:
            queue_present = True
            # print(queue_present)
        
        if self.v == 0:
            self.skip_us_control = True

        
        
        if self.ECO_CACC:
            if (lead == None or lead.ECO_CACC == False) and self.v <= 1.25:
                consider_jerk = True
                max_jerk = 1.3
            else:
                consider_jerk = False
                max_jerk = 1.3
            # FRModel.a, FRModel.b, FRModel.d = 0.1, 10, 0.1  # automated driving parameters
        else:
            consider_jerk = True
            max_jerk = 1.3
            # FRModel.a, FRModel.b, FRModel.d = 0.74375, 1.0, 0.1  #driver behavior parameters (HDV)



        used_cf_model = 'FR'
        # used_cf_model = 'RPA'
        # self.look_ahead_distance =  max(40, FRModel.calc_desired_S(self.v, 0, 99999, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c))
        
        # # Accounting for current signal state
        # arrival_t = self.sim.t
        # signal_state = self.current_lane.traffic_signal_state


        # print(current_signal_state, self.current_lane.future_traffic_signal_state(self.sim.t + (delta_x_signal/self.v)))
        # if self.sim.t >= 111 and self.id == 13:
        #     stop = 1

        if self.id == 17:
            stop = 1
        # print(self.sim.t)
        if lead:
            if used_cf_model == 'FR': a_cf = FRModel.get_a(self.a_prev, self.v, lead_spacing, lead.v, current_segment, consider_jerk, max_jerk)
            elif used_cf_model == 'RPA': a_cf = RPAModel.get_a(self.v, lead.v, lead_spacing, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
            a_signal = 6
            a_Eco_CACC_DS = 6

            if (signal_state == 'red' or queue_present) and self.current_lane.has_traffic_signal: 
                if used_cf_model == 'FR': a_signal = FRModel.get_a(self.a_prev, self.v, delta_x_signal, 0, current_segment, consider_jerk, max_jerk)
                elif used_cf_model == 'RPA': a_signal = RPAModel.get_a(self.v, 0, delta_x_signal, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)

                ### ECO-CACC UPSTREAM ###
                if self.ECO_CACC and DTI < self.ECO_CACC_US_cont_dist and not self.skip_us_control:
                    a_Eco_CACC_US = self.ECO_CACC.get_a_EcoCACC_US(self, a_cf, DTI, signal_state, queue_present)
                    a_signal = a_Eco_CACC_US

                else:
                    if used_cf_model == 'FR': a_signal = FRModel.get_a(self.a_prev, self.v, delta_x_signal, 0, current_segment, consider_jerk, max_jerk)
                    elif used_cf_model == 'RPA': a_signal = RPAModel.get_a(self.v, 0, delta_x_signal, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                
            elif (signal_state == 'yellow') and self.current_lane.has_traffic_signal:
                sig = current_segment.lanes[0].traffic_signal
                i = sig.current_phase_index[current_segment.lanes[0].traffic_signal_ring_no]

                a_max = Dynamics.get_max_acceleration(self.v, current_segment.grade)
                v_at_stopbar = min(np.sqrt(max(0, self.v**2 + 2 * DTI * a_max)), current_segment.u_f)
                time_to_cross = (DTI) * 2 / max(0.01, (v_at_stopbar + self.v - 0.1))
                
                if current_segment.lanes[0].traffic_signal_ring_no == 0 and sig.NEMA == True:
                    phase_length_ = sig.phase_length[:4]
                elif sig.NEMA == True:
                    i = i - 4
                    phase_length_ = sig.phase_length[4:]
                else: phase_length_ = sig.phase_length

                if sig.actuated:
                    curr_phase_index = sig.current_phase_index.index(self.current_lane.traffic_signal_phase[0])
                    remaining_yellow_time = sig.curr_phase_end[curr_phase_index] - current_time - sig.all_red_time
                else: remaining_yellow_time =  (sum(phase_length_[:i+1]) - sig.all_red_time) - (current_time % sig.cycle_length)
                if time_to_cross > remaining_yellow_time:
                    if self.ECO_CACC and DTI < self.ECO_CACC_US_cont_dist and not self.skip_us_control:
                        a_Eco_CACC_US = self.ECO_CACC.get_a_EcoCACC_US(self, a_cf, DTI, signal_state, queue_present)
                        a_signal = a_Eco_CACC_US
                        if a_Eco_CACC_US > 6 or self.v < 0.1 or self.v_prev < 0.1:
                            a_signal = 0
                    else:
                        if used_cf_model == 'FR': a_signal = FRModel.get_a(self.a_prev, self.v, delta_x_signal, 0, current_segment, consider_jerk, max_jerk)
                        elif used_cf_model == 'RPA': a_signal = RPAModel.get_a(self.v, 0, delta_x_signal, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                else:
                    if DTI > 0: DTD = max(0, self.ECO_CACC_DS_cont_dist + DTI)
                    else: DTD = max(0, self.ECO_CACC_DS_cont_dist - (self.x_cumm - self.x_last_stopbar))
                    if self.ECO_CACC and DTD > 0 and lead.ECO_CACC == False: 
                    # if self.ECO_CACC and DTD > 0:
                        a_Eco_CACC_DS = self.ECO_CACC.get_a_EcoCACC_DS(self, DTD)
                    else: a_Eco_CACC_DS = a_cf

            elif (signal_state == 'green') or not self.current_lane.has_traffic_signal: #if no signal

                if DTI > 0: DTD = max(0, self.ECO_CACC_DS_cont_dist + DTI)
                else: DTD = max(0, self.ECO_CACC_DS_cont_dist - (self.x_cumm - self.x_last_stopbar))

                if self.ECO_CACC and DTD > 0 and lead.ECO_CACC == False: 
                # if self.ECO_CACC and DTD > 0: 
                    a_Eco_CACC_DS = self.ECO_CACC.get_a_EcoCACC_DS(self, DTD)
                else: a_Eco_CACC_DS = a_cf

            self.a = min(a_cf, a_signal, a_Eco_CACC_DS)
            # if self.id == 30 and min(a_cf, a_signal, a_Eco_CACC_DS) == a_cf:
            #     print(self.sim.t, 'cf', a_cf, a_Eco_CACC_DS)
            # elif self.id == 30 and min(a_cf, a_signal, a_Eco_CACC_DS) == a_Eco_CACC_DS:
            #     print(self.sim.t, 'ECO-CACC', a_cf, a_Eco_CACC_DS)
        else:
            virtual_leader_speed  = current_segment.u_f - 0.0001
            a_cf = 6
            a_signal = 6
            a_Eco_CACC_DS = 6

            if (signal_state == 'red' or queue_present) and self.current_lane.has_traffic_signal:
                ### ECO-CACC UPSTREAM ###
                if self.ECO_CACC and DTI < self.ECO_CACC_US_cont_dist and not self.skip_us_control:
                    a_Eco_CACC_US = self.ECO_CACC.get_a_EcoCACC_US(self, a_cf, DTI, signal_state, queue_present)
                    a_signal = a_Eco_CACC_US
                    if a_Eco_CACC_US > 6 or self.v < 0.1 or self.v_prev < 0.1:
                        a_signal = 0                    
                else:
                    if used_cf_model == 'FR': 
                        a_cf = FRModel.get_a(self.a_prev, self.v, 99999, virtual_leader_speed, current_segment, consider_jerk, max_jerk)
                        a_signal = FRModel.get_a(self.a_prev, self.v, delta_x_signal, 0, current_segment, consider_jerk, max_jerk)
                    elif used_cf_model == 'RPA': 
                        a_cf = RPAModel.get_a(self.v, virtual_leader_speed, 99999, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                        a_signal = RPAModel.get_a(self.v, 0, delta_x_signal, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)

            
            elif (signal_state == 'yellow') and self.current_lane.has_traffic_signal:
                sig = current_segment.lanes[0].traffic_signal
                i = sig.current_phase_index[current_segment.lanes[0].traffic_signal_ring_no]

                a_max = Dynamics.get_max_acceleration(self.v, current_segment.grade)
                v_at_stopbar = min(np.sqrt(max(0, self.v**2 + 2 * DTI * a_max)), current_segment.u_f)
                time_to_cross = (DTI) * 2 / max(0.01, (v_at_stopbar + self.v - 0.1))

                if current_segment.lanes[0].traffic_signal_ring_no == 0 and sig.NEMA == True:
                    phase_length_ = sig.phase_length[:4]
                elif sig.NEMA == True:
                    i = i - 4
                    phase_length_ = sig.phase_length[4:]
                else: phase_length_ = sig.phase_length

                if sig.actuated:
                    curr_phase_index = sig.current_phase_index.index(self.current_lane.traffic_signal_phase[0])
                    remaining_yellow_time = sig.curr_phase_end[curr_phase_index] - current_time - sig.all_red_time
                else: remaining_yellow_time =  (sum(phase_length_[:i+1]) - sig.all_red_time) - (current_time % sig.cycle_length)
                if time_to_cross > remaining_yellow_time:
                    if self.ECO_CACC and DTI < self.ECO_CACC_US_cont_dist and not self.skip_us_control:
                        a_Eco_CACC_US = self.ECO_CACC.get_a_EcoCACC_US(self, a_cf, DTI, signal_state, queue_present)
                        a_signal = a_Eco_CACC_US
                        if a_Eco_CACC_US > 6 or self.v < 0.1 or self.v_prev < 0.1:
                            a_signal = 0
                    else:
                        if used_cf_model == 'FR': 
                            a_cf = FRModel.get_a(self.a_prev, self.v, lead_spacing, virtual_leader_speed, current_segment, consider_jerk, max_jerk)
                            a_signal = FRModel.get_a(self.a_prev, self.v, delta_x_signal, 0, current_segment, consider_jerk, max_jerk)
                        elif used_cf_model == 'RPA':
                            a_cf = RPAModel.get_a(self.v, virtual_leader_speed, lead_spacing, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                            a_signal = RPAModel.get_a(self.v, 0, delta_x_signal, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)

                else:
                    
                    if DTI > 0: DTD = max(0, self.ECO_CACC_DS_cont_dist + DTI)
                    else: DTD = max(0, self.ECO_CACC_DS_cont_dist - (self.x_cumm - self.x_last_stopbar))

                    if self.ECO_CACC and DTD > 0: 
                        a_Eco_CACC_DS = self.ECO_CACC.get_a_EcoCACC_DS(self, DTD)
                    else:
                        if used_cf_model == 'FR': a_cf = FRModel.get_a(self.a_prev, self.v, lead_spacing, virtual_leader_speed, current_segment, consider_jerk, max_jerk)
                        elif used_cf_model == 'RPA': a_cf = RPAModel.get_a(self.v, virtual_leader_speed, lead_spacing, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                        a_Eco_CACC_DS = a_cf
                    
            elif (signal_state == 'green') or not self.current_lane.has_traffic_signal: #if green or no signal
                #accounting for start loss
                if self.sim.t == 25:
                    stop = 1
                # print(self.sim.t, self.current_lane.last_red_indication_t)
                
                perception_reaction_t = FRModel.calc_tau(self.v, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c)
                # perception_reaction_t = 1.5
                if self.sim.t - self.current_lane.last_red_indication_t <= perception_reaction_t and self.veh_class == 1 and self.v == 0: 
                    a_signal = 0
                    
                if DTI > 0: DTD = max(0, self.ECO_CACC_DS_cont_dist + DTI)
                else: DTD = max(0, self.ECO_CACC_DS_cont_dist - (self.x_cumm - self.x_last_stopbar))

                if self.ECO_CACC and DTD > 0: 
                    a_Eco_CACC_DS = self.ECO_CACC.get_a_EcoCACC_DS(self, DTD)
                    a_signal = a_Eco_CACC_DS
                    if a_Eco_CACC_DS == 6:
                        if used_cf_model == 'FR': a_cf = FRModel.get_a(self.a_prev, self.v, lead_spacing, virtual_leader_speed, current_segment, consider_jerk, max_jerk)
                        elif used_cf_model == 'RPA': a_cf = RPAModel.get_a(self.v, virtual_leader_speed, lead_spacing, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                else:
                    if used_cf_model == 'FR': a_cf = FRModel.get_a(self.a_prev, self.v, lead_spacing, virtual_leader_speed, current_segment, consider_jerk, max_jerk)
                    elif used_cf_model == 'RPA': a_cf = RPAModel.get_a(self.v, virtual_leader_speed, lead_spacing, current_segment.u_f, current_segment.q_c, current_segment.k_j, current_segment.u_c, dt, current_segment.grade)
                    a_Eco_CACC_DS = 6
                
                
            self.a = min(a_cf, a_signal, a_Eco_CACC_DS)
            # if self.id == 30 and min(a_cf, a_signal, a_Eco_CACC_DS) == a_cf:
            #     print(self.sim.t, 'cf', a_cf, a_Eco_CACC_DS)
            # elif self.id == 30 and min(a_cf, a_signal, a_Eco_CACC_DS) == a_Eco_CACC_DS:
            #     print(self.sim.t, 'ECO-CACC', a_cf, a_Eco_CACC_DS)
            
        
        
        self.current_spacing = lead_spacing

        if self.stopped: 
            self.a = 0

        # uncomment to store traj data
        self.traj_array_veh.append([current_time, self.id, self.veh_class, self.veh_type, self.x_cumm, self.current_seg.id,
                                                                    self.current_lane.id, self.x, self.v*3.6, self.current_spacing])

    
    def stop(self):
        self.stopped = True

    def unstop(self):
        self.stopped = False

    def slow(self, v):
        self.v_max = v

    # def unslow(self):
    #     self.v_max = self._v_max