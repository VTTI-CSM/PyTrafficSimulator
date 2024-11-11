import numpy as np
from src.core.models.motion import Motion
# from numpy import random
import random
import pandas as pd

class TrafficSignal:
    def __init__(self, sim, lanes, config={}):
        # Initialize segments
        self.lanes = lanes
        self.sim = sim
        random.seed(self.sim.rseed)
        # Set default configuration
        self.set_default_config()

        # Update configuration
        for attr, val in config.items():
            setattr(self, attr, val)
        # Calculate properties
        self.init_properties(config)

    def set_default_config(self):
        self.id = None
        self.NEMA = False
        self.cycle = [(True, False, False, False), (False, True, False, False), (False, False, True, False), (False, False, False, True)]
        # self.cycle = [(True, False, True, False), (False, True, False, True)]
        self.number_of_phases = len(self.cycle)
        self.phase_length = np.repeat(10,self.number_of_phases).tolist()
        self.sig_stat_prev =  np.repeat('r',self.number_of_phases).tolist()
        self.sig_stat =  np.repeat('r',self.number_of_phases).tolist()
        self.current_phase_index = [0]
        self.last_t = 0
        self.cycle_length = 0
        self.NB_obj = None
        self.NB_Activation = False
        self.yellow_time = None
        self.all_red_time = None
        self.switch = 1
        self.phase_start = [0]
        self.prev_phase_start = [0]
        self.signal_indication_array = np.empty((0, 4))
        self.queue_model = QueueModel(self)
        

    def init_properties(self, config):
        if self.NB_Activation: 
            self.phase_length = [config['update_period_lt'] + config['yellow_time'] + config['all_red_time'],
                                 config['update_period_thr'] + config['yellow_time'] + config['all_red_time'],
                                 config['update_period_lt'] + config['yellow_time'] + config['all_red_time'],
                                 config['update_period_thr'] + config['yellow_time'] + config['all_red_time']]
            self.NB_obj = NBController(self, config={'update_period_lt': config['update_period_lt'],
                                                                         'update_period_thr': config['update_period_thr'],
                                                                         'threat_points': config['threat_points']})

        self.cycle_length = sum(self.phase_length)
        self.number_of_phases = len(self.cycle)
        self.sig_stat_prev =  np.repeat('r',self.number_of_phases).tolist()
        self.sig_stat =  np.repeat('r',self.number_of_phases).tolist()

        for phase_no in range(len(self.lanes)):
            for lane in self.lanes[phase_no]:
                lane.set_traffic_signal(self, phase_no)
                lane.segment.has_traffic_signal = True
                lane.segment.traffic_signal = self

    @property
    def current_cycle(self):
        return self.cycle[self.current_phase_index[0]]
    

    def get_time_to_switch_det(self, t, phase_index):
        curr_t_cycle = t % self.cycle_length
        switching_time = sum(self.phase_length[:self.current_phase_index[0]+1])
        if phase_index[0] == self.current_phase_index[0]: 
            result = switching_time - curr_t_cycle + sum(self.phase_length) - self.phase_length[self.current_phase_index[0]] 
        else:
            result = switching_time - curr_t_cycle   
        return result


    def get_time_to_switch_stoch0(self, t, phase_index):
        curr_t_cycle = t % self.cycle_length
        switching_time = sum(self.phase_length[:self.current_phase_index[0]+1])
        if phase_index[0] == self.current_phase_index[0]: 
            ttc = switching_time - curr_t_cycle + sum(self.phase_length) - self.phase_length[self.current_phase_index[0]] 
        else:
            ttc = switching_time - curr_t_cycle
        

        invalid_stoch_value = True
        while invalid_stoch_value:
            df = self.sim.ttg_prediction_df
            try: error = random.choice(list(df[df.ytrue==int(ttc)].error))
            except:
                try: error = random.choice(list(df[df.ytrue==int(ttc+1)].error))
                except: error = random.choice(list(df[df.ytrue==df.ytrue.max()].error))
            # error = random.normal(0, 1, 1)[0]
            if ttc + error >= 0:
                ttc = ttc + error
                invalid_stoch_value = False
        return ttc
    
    def get_time_to_switch_stoch(self, t, phase_index):
        curr_t_cycle = t % self.cycle_length
        switching_time = sum(self.phase_length[:self.current_phase_index[0]+1])
        if phase_index[0] == self.current_phase_index[0]: 
            ttc = switching_time - curr_t_cycle + sum(self.phase_length) - self.phase_length[self.current_phase_index[0]] 
        else:
            ttc = switching_time - curr_t_cycle
        

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
        for phase in range(0, self.number_of_phases):
            if phase == self.current_phase_index[0]:
                # curr_t_phase = sim.t - self.phase_start
                curr_t_phase = sim.t - self.prev_phase_start[0]
                if (curr_t_phase > self.phase_length[phase] - self.all_red_time):
                    self.sig_stat[phase] = 'r'
                elif (curr_t_phase > self.phase_length[phase] - self.yellow_time - self.all_red_time):
                    self.sig_stat[phase] = 'y'
                else: self.sig_stat[phase] = 'g'
            else: self.sig_stat[phase] = 'r'

    def store_data(self, sim):
        # print(self.sig_stat)
        for phase_no in range(0, self.number_of_phases):
            new_array = [[round(sim.t, 3), self.id, phase_no+1, self.sig_stat[phase_no]]]
            self.signal_indication_array = np.concatenate((self.signal_indication_array, new_array))

    def store_data_nb(self, sim):
        for phase in range(0, self.number_of_phases):
            if phase == self.current_phase_index[0]:
                curr_t_phase = sim.t - self.phase_start[0]
                if (curr_t_phase > self.phase_length[phase] - self.all_red_time):
                    Signal_Status = 'r'
                elif (curr_t_phase > self.phase_length[phase] - self.yellow_time - self.all_red_time):
                    Signal_Status = 'y'
                else: Signal_Status = 'g'
            else: Signal_Status = 'r'

        for phase in range(0, self.number_of_phases):
            self.signal_indication_array = np.concatenate((self.signal_indication_array, [[round(sim.t, 3), self.id, phase+1, Signal_Status]]))

    def update(self, sim):
        self.switch = 1
        prev_index = self.current_phase_index[0]
        self.current_phase_index[0] = next(i for i, _ in enumerate(self.phase_length) if (sim.t % sum(self.phase_length)) < sum(self.phase_length[:i+1]))
        
        if prev_index != self.current_phase_index[0]:
            self.phase_start[0] = sim.t
            self.prev_phase_start[0] = self.phase_start[0]
        self.set_sig_stat(sim)
        
        if self.sig_stat_prev != self.sig_stat:
            self.store_data(sim)
    

    def update_actuated(self, sim):
        pass

