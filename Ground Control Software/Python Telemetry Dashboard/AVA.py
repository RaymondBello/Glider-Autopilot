import time

class AVA:
    '''All-Vehicle Avionics'''
    def __init__(self, ip, state, pool):
        self.ip = ip
        self.current_state = state
        self.state_pool = pool
        self.data = []
        self.previous = 0
        self.sample_rate = 0
        self.accel = []
        self.gyro = []
        self.mag = []
        self.orientation = []
        self.position = []

    def update(self, serial_array:list, current_state:int):
        '''Takes array:list, current_state:int as arguments returns new serial data to plot'''
        self.current_state = current_state
        self.data = serial_array

        ''' Swapping x&y for my calculation (To maintain North.East.Down Convention)'''

        if (len(self.data) == 19):
            self.update_variables()

            



            self.previous = time.perf_counter()
            # print(f"{format(1/self.t_delta_ms,'.2f')} hz")
        
        return self.data
    
    @staticmethod
    def swapIdx_NED(data: list):
        """
        Swaps X&Y axis to Maintain North East Down Convention
        """
        # Accel
        data[0], data[1] = data[1], data[0]
        # Gyro
        data[3], data[4] = data[4], data[3]
        # Mag
        data[6], data[7] = data[7], data[6]

        return data

    def update_variables(self):
        """
        Updates all the internal variables used for calculation
        """
        self.t_delta_ms = time.perf_counter() - self.previous
        self.sample_rate = 1 / self.t_delta_ms
        self.data = self.swapIdx_NED(self.data)
        self.accel = self.data[:3]
        self.gyro = self.data[3:6]
        self.mag = self.data[6:9]
        print(self.data[6:9])

        
