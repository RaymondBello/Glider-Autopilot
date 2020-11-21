

class AVA:
    '''All-Vehicle Avionics'''

    def __init__(self, ip, state, pool):
        self.ip = ip
        self.current_state = state
        self.state_pool = pool
        self.data = []

    def update(self, serial_array:list, current_state:int):
        '''Takes array:list, current_state:int as arguments returns new serial data to plot'''
        self.current_state = current_state
        self.data = serial_array

        print(self.data[5:9])

        return self.data

