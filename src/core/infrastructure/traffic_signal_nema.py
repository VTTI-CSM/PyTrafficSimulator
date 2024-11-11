import numpy as np
import random


class TrafficSignalNEMA:
    def __init__(self, sim, config={}):
        # Set default configuration
        self.set_default_config(sim)

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)
        # Calculate properties
        self.init_properties(config)

    def set_default_config(self, sim):
        self.id = None
        self.NEMA = True
        self.sim = sim

        self.yellow_time = None
        self.all_red_time = None
        self.phase_length_ring1 = []
        self.phase_length_ring2 = []
        self.lanes = None
        self.cycle_length = None



        self.number_of_phases = 8
        self.current_phase_index = [0, int(self.number_of_phases/2)]
        self.next_phase_index = [None, None]
        self.sig_stat_prev =  ['g', 'r', 'r', 'r', 'g', 'r', 'r', 'r']
        self.sig_stat =  ['g', 'r', 'r', 'r', 'g', 'r', 'r', 'r']
        self.signal_indication_array = np.empty((0, int(self.number_of_phases/2)))
        
        
        self.switch = 1
        self.phase_start = [0, 0]
        self.prev_phase_start = [0, 0]

        self.NB_obj = None
        self.NB_Activation = False

        random.seed(sim.rseed)



        # =======================================================================================
        # Actuated Controller variables
        # =======================================================================================
        self.actuated = None
        self.actuations = []
        self.phase_time = []
        self.non_conflicts = {}
        self.ring_alternative = {}
        self.curr_phase_start = [0, 0]

        self.min_green = [10, 10, 10, 10, 10, 10, 10, 10]       # Table 5-3, 5-4
        self.max_green = [20, 20, 20, 20, 20, 20, 20, 20]       # Table 5-5, 5-6
        self.min_recall = None
        self.passage_time = 3
        self.phase_combinations = None
        self.curr_phase_end = [3, 3]
        self.check_switch = True
        # =======================================================================================


    def init_properties(self, config):
        self.current_phase_index = [0, int(self.number_of_phases/2)]
        self.sig_stat_prev = ['g' if i == 0 or i == int(self.number_of_phases/2) else 'r' for i in range(self.number_of_phases)]
        
        # ['g', 'r', 'r', 'r', 'g', 'r', 'r', 'r']


        
        if self.NB_Activation: 
            self.phase_length = np.repeat(config['update_period'] + config['yellow_time'] + config['all_red_time'], self.number_of_phases).tolist()
            self.NB_obj = NBControllerNEMA(self, config={'update_period': config['update_period'], 'threat_points': config['threat_points'], 'threat_pt_dist': config['threat_pt_dist']})

        self.cycle_length = sum(self.phase_length_ring1)
        self.phase_length = self.phase_length_ring1 + self.phase_length_ring2
        self.intergreen_time = self.all_red_time + self.yellow_time


        for phase_no in range(len(self.lanes)):
            for lane in self.lanes[phase_no]:
                lane.set_traffic_signal(self, phase_no)
                lane.segment.has_traffic_signal = True
                lane.segment.traffic_signal = self
        
        # =======================================================================================
        # Actuated Controller variables
        # =======================================================================================
        self.actuations = [[False,0] for i in range(self.number_of_phases)]
        self.phase_time = [0 for i in range(self.number_of_phases)]

        if self.number_of_phases == 8: 
            self.phase_combinations = [(0,4), (0,5), (1,4), (1,5), (2,6), (2,7), (3,6), (3,7)]
            self.non_conflicts = {0: [4,5], 1: [4,5], 2: [6,7], 3: [6,7], 4: [0,1], 5: [0,1], 6: [2,3], 7: [2,3]}
            self.conflicts = {0: [1,2,3,6,7], 1: [0,2,3,6,7], 2: [0,1,4,5,6], 3: [0,1,4,5,7], 4: [2,3,5,6,7], 5: [2,3,4,6,7], 6: [0,1,4,5,7], 7: [0,1,4,5,6]}
            self.next_phase_actuated = {(0,4):[(0,5), (1,5)], (0,5):[(1,5)], (1,4):[(1,5)], (1,5):[(2,6)], (2,6):[(2,7), (3,6)], (2,7):[(3,7)], (3,6):[(3,7)], (3,7):[(0,4)]}

            self.ring_alternative = {0: 1, 1: 0, 2: 3, 3: 2, 4: 5, 5: 4, 6: 7, 7: 6}
        elif self.number_of_phases == 4: 
            self.phase_combinations = [(0,2), (1,3)]
            self.non_conflicts = {0: [2], 1: [3], 2: [0], 3: [1]}
            self.conflicts = {0: [1,3], 1: [0,2], 2: [0,3], 3: [1,2]}
        # =======================================================================================
    
    def get_cycle(self, curr_phase_index = (0,4)):
        output = np.repeat(False,self.number_of_phases).tolist()
        output[curr_phase_index[0]] = True
        output[curr_phase_index[1]] = True
        return output

    @property
    def current_cycle(self):
        '''Returns the current phase status - False is red, True is green or intergreen.'''
        return self.get_cycle(self.current_phase_index)
    
    def get_time_to_switch_det(self, t, phase_index):
        curr_t_cycle = t % self.cycle_length
        switching_time = sum(self.phase_length[:self.current_phase_index[0]+1])

        if phase_index[0] == self.current_phase_index[0] or phase_index[0] == self.current_phase_index[1]: 
            result = switching_time - curr_t_cycle + sum(self.phase_length)/2 - self.phase_length[phase_index[0]] 
        else:
            result = switching_time - curr_t_cycle   
        return result
    
    def get_time_to_switch_stoch(self, t, phase_index):
        ttc = self.get_time_to_switch_det(t, phase_index)
        
        invalid_stoch_value = True
        err_dict = self.sim.ttg_prediction_dict
        while invalid_stoch_value:
            try: error_list = err_dict[int(ttc)]
            except:
                try: error_list = err_dict[int(ttc+1)]
                except: error_list = err_dict[max(err_dict.keys())]
            
            average_error = np.mean(error_list)
            std_error = np.std(error_list)
            error, ub, lb = random.choice(error_list), average_error+1.96*std_error, average_error-1.96*std_error

            if ttc + error >= 0:
                ttc = ttc + error
                invalid_stoch_value = False
        return ttc
    

    def set_sig_stat(self, sim):
        self.sig_stat_prev = self.sig_stat.copy()
        self.sig_stat = np.repeat('r',self.number_of_phases).tolist()

        

        if not self.NB_obj and not self.actuated:
            curr_t_phase = (sim.t - self.prev_phase_start[0], sim.t - self.prev_phase_start[1])

            # setting ring 1
            if (curr_t_phase[0] > self.phase_length_ring1[self.current_phase_index[0]] - self.all_red_time):
                self.sig_stat[self.current_phase_index[0]] = 'r'
            elif (curr_t_phase[0] > self.phase_length_ring1[self.current_phase_index[0]] - self.yellow_time - self.all_red_time):
                self.sig_stat[self.current_phase_index[0]] = 'y'
            else: self.sig_stat[self.current_phase_index[0]] = 'g'

            # setting ring 2
            if (curr_t_phase[1] > self.phase_length_ring2[self.current_phase_index[1]-4] - self.all_red_time):
                self.sig_stat[self.current_phase_index[1]] = 'r'
            elif (curr_t_phase[1] > self.phase_length_ring2[self.current_phase_index[1]-4] - self.yellow_time - self.all_red_time):
                self.sig_stat[self.current_phase_index[1]] = 'y'
            else: self.sig_stat[self.current_phase_index[1]] = 'g'
        elif self.actuated:
            if (sim.t >  self.curr_phase_end[0] - self.all_red_time) and (self.next_phase_index[0] != self.current_phase_index[0]):
                self.sig_stat[self.current_phase_index[0]] = 'r'
            elif (sim.t > self.curr_phase_end[0] - self.intergreen_time) and (self.next_phase_index[0] != self.current_phase_index[0]):
                self.sig_stat[self.current_phase_index[0]] = 'y'
            else: self.sig_stat[self.current_phase_index[0]] = 'g'

            # setting ring 2
            if (sim.t >  self.curr_phase_end[1] - self.all_red_time) and (self.next_phase_index[1] != self.current_phase_index[1]):
                self.sig_stat[self.current_phase_index[1]] = 'r'
            elif (sim.t > self.curr_phase_end[1] - self.intergreen_time) and (self.next_phase_index[1] != self.current_phase_index[1]):
                self.sig_stat[self.current_phase_index[1]] = 'y'
            else: self.sig_stat[self.current_phase_index[1]] = 'g'

        else:
            if (sim.t >  self.phase_start[0] - self.all_red_time) and (self.next_phase_index[0] != self.current_phase_index[0]):
                self.sig_stat[self.current_phase_index[0]] = 'r'
            elif (sim.t > self.phase_start[0] - self.intergreen_time) and (self.next_phase_index[0] != self.current_phase_index[0]):
                self.sig_stat[self.current_phase_index[0]] = 'y'
            else: self.sig_stat[self.current_phase_index[0]] = 'g'

            # setting ring 2
            if (sim.t >  self.phase_start[1] - self.all_red_time) and (self.next_phase_index[1] != self.current_phase_index[1]):
                self.sig_stat[self.current_phase_index[1]] = 'r'
            elif (sim.t > self.phase_start[1] - self.intergreen_time) and (self.next_phase_index[1] != self.current_phase_index[1]):
                self.sig_stat[self.current_phase_index[1]] = 'y'
            else: self.sig_stat[self.current_phase_index[1]] = 'g'
        

    def store_data(self, sim):
        '''
        Store the signal indication data for signal output file
        '''
        for phase_no in range(0, self.number_of_phases):
            self.signal_indication_array = np.concatenate((self.signal_indication_array, [[round(sim.t, 0), self.id, phase_no+1, self.sig_stat[phase_no]]]))

    # def store_data_nb(self, sim):
    #     for phase in range(0, self.number_of_phases):
    #         if phase == self.current_phase_index:
    #             curr_t_phase = sim.t - self.phase_start
    #             if (curr_t_phase > self.phase_length[phase] - self.all_red_time):
    #                 Signal_Status = 'r'
    #             elif (curr_t_phase > self.phase_length[phase] - self.yellow_time - self.all_red_time):
    #                 Signal_Status = 'y'
    #             else: Signal_Status = 'g'
    #         else: Signal_Status = 'r'

    #     for phase in range(0, self.number_of_phases):
    #         self.signal_indication_array = np.concatenate((self.signal_indication_array, [[round(sim.t, 3), self.id, phase+1, Signal_Status]]))


    def update_fixed(self, sim):
        self.switch = 1
        prev_index = self.current_phase_index.copy()
        self.current_phase_index = [next(i for i, _ in enumerate(self.phase_length_ring1) if (sim.t % self.cycle_length) < sum(self.phase_length_ring1[:i+1])),
                                    next(i for i, _ in enumerate(self.phase_length_ring2) if (sim.t % self.cycle_length) < sum(self.phase_length_ring2[:i+1])) + int(self.number_of_phases/2)]
        
        for i in range(2):
            if prev_index[i] != self.current_phase_index[i]:
                self.phase_start[i] = sim.t
                self.prev_phase_start[i] = self.phase_start[i]
        
        self.set_sig_stat(sim)
            
        if self.sig_stat_prev != self.sig_stat or sim.t == 0:
            self.store_data(sim)
    
    #########################################################################################
    # =======================================================================================
    # Actuated Controller Functions
    # =======================================================================================
    def extend(self):
        pass

    def gap_out(self, switch):
        if False in switch and self.current_phase_index[switch.index(True)] in [1,5,3,7]:  
            return self.current_phase_index
        else:
            new_phase = [None, None]
            for i, k in enumerate(switch):
                if k == True: 
                    new_phase[i] = self.current_phase_index[i] + 1
                else: new_phase[i] = self.current_phase_index[i]
            
            if new_phase == [2,5]:
                stop=1
            
            if new_phase[0] > 3: new_phase[0] = 0
            if new_phase[1] > 7: new_phase[1] = 4

            
            if tuple(new_phase) not in self.phase_combinations: 
                new_phase = self.next_phase_actuated[tuple(self.current_phase_index)][0]

            new_phase = self.skip_phase(new_phase)
            self.check_switch = False
            return new_phase

    def max_out(self, switch):
        new_phase = [None, None]
        for i, k in enumerate(switch):
            if k == True: 
                new_phase[i] = self.current_phase_index[i] + 1
            else: new_phase[i] = self.current_phase_index[i]
        if new_phase == [2,5]:
            stop=1
        
        if new_phase[0] > 3: new_phase[0] = 0
        if new_phase[1] > 7: new_phase[1] = 4

        if tuple(new_phase) not in self.phase_combinations: 
            new_phase = self.next_phase_actuated[tuple(self.current_phase_index)][0]

        new_phase = self.skip_phase(new_phase)
        self.check_switch = False
        return list(new_phase)


    def skip_phase(self, new_phase):
        new_phase2 = list(new_phase)

        
        conflicts = set(self.conflicts[new_phase[0]] + self.conflicts[new_phase[1]])
        conflicting = False
        for i in conflicts:
            if i in self.current_phase_index:
                continue
            elif self.actuations[i][0] == True:
                conflicting = True
                break
        
        if conflicting:
            if self.actuations[new_phase[0]][0] == False and self.actuations[new_phase[1]][0] == False and self.phase_time[new_phase[0]] > self.min_green[new_phase[0]] and self.phase_time[new_phase[1]] > self.min_green[new_phase[1]]:
                new_phase2 = [x+1 for x in new_phase2]
            elif self.actuations[new_phase[0]][0] == False and self.phase_time[new_phase[0]] > self.min_green[new_phase[0]]:
                new_phase2[0] = new_phase2[0]+1
            elif self.actuations[new_phase[1]][0] == False and self.phase_time[new_phase[1]] > self.min_green[new_phase[1]]:
                new_phase2[1] = new_phase2[1]+1
            
            if new_phase2[0] > 4: new_phase2[0] = 0
            if new_phase2[1] > 7: new_phase2[1] = 4


            if tuple(new_phase2) not in self.phase_combinations: 
                new_phase2 = self.next_phase_actuated[tuple(self.current_phase_index)][0]
                # new_phase2 = self.skip_phase(new_phase2)
        
        return new_phase2
        

    def reset_max_green(self):
        pass



    def update_actuated(self, sim):     
        self.switch = 1

        if tuple(self.current_phase_index) not in self.phase_combinations:
            stop = 1

        #updating phase time and actuations
        for index, phase in enumerate(self.lanes):
            for lane in phase:
                # print(lane.traffic_signal_phase)
                conflicting_actuation = False
                for conflict_index in self.conflicts[index]:
                    if self.actuations[conflict_index][0] == True:
                        conflicting_actuation = True
                        break
                
                if lane.detector.detector_call or (not conflicting_actuation and index in self.current_phase_index): 
                    self.actuations[index] = [True, sim.t]
                else: self.actuations[index][0] = False

            if index in self.current_phase_index:
                self.phase_time[index] += sim.dt
            else: self.phase_time[index] = 0
            
        # print(self.actuations, self.phase_time)
        max_out_switch = [False, False]
        gap_out_switch = [False, False]
        extend_switch = [False, False]
        for i, phase_index in enumerate(self.current_phase_index):
            if self.phase_time[phase_index] > self.max_green[phase_index]:
                max_out_switch[i] = True
            elif sim.t - self.actuations[phase_index][1] > self.passage_time and self.phase_time[phase_index] > self.min_green[phase_index]:
                for conflict_index in self.conflicts[phase_index]:
                    if self.actuations[conflict_index][0] == True:
                        gap_out_switch[i] = True
            else: 
                extend_switch[i] = True
        
        if self.check_switch:
            if True in max_out_switch:
                new_phase = self.max_out(max_out_switch)
            elif True in gap_out_switch:
            # elif gap_out_switch == [True, True]:
                new_phase = self.gap_out(gap_out_switch)
            else:
                new_phase = self.current_phase_index
            self.next_phase_index = new_phase

            for i in range(2):
                if self.current_phase_index[i] != self.next_phase_index[i]:
                    self.curr_phase_end[i] = sim.t + self.intergreen_time
        

        if tuple(self.next_phase_index) not in self.phase_combinations:
            stop = 1
        


        for i in range(2):
            if self.current_phase_index[i] == self.next_phase_index[i]:
                self.curr_phase_end[i] = max(min(sim.t + self.passage_time, self.curr_phase_start[i] + self.max_green[self.current_phase_index[i]]), self.curr_phase_start[i] + self.min_green[self.current_phase_index[i]]) + self.intergreen_time
            
            if sim.t >= self.curr_phase_end[i] and self.current_phase_index[i] != self.next_phase_index[i]:
                self.current_phase_index[i] = self.next_phase_index[i]
                self.phase_start[i] = self.curr_phase_end[i]
                self.curr_phase_start[i] = self.phase_start[i]
                self.check_switch = True
                
        
        self.set_sig_stat(sim)
            
        if self.sig_stat_prev != self.sig_stat or sim.t == 0:
            self.store_data(sim)
        pass
    #########################################################################################

