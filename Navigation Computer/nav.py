import os
import sys
import logging
import logging.config
from src.module_list import *

# Set up environment variables
os.environ["NAV_REPO_PATH"] = os.path.dirname(os.path.realpath(__file__))
NAV_REPO_PATH = os.environ["NAV_REPO_PATH"]

# Setup Logging config file  
logging.config.fileConfig(f'{NAV_REPO_PATH}/config/logging.conf')

class NavigationComputer:
    
    # Flight controller mode constants
    ERROR = 0
    UNINITIALIZED = 1
    INITIALIZED = 2
    ACTIVE = 4
    IDLE = 8
    REBOOT = 16
    
    def __init__(self, log_name='NavigationSystem', args = sys.argv):
        # runtime args to be used later
        self.sys_args = args
        
        # Setup logger
        self.log = logging.getLogger(log_name)
        self.log.info('Initializing Navigation Computer')

        # Setup serial handler
        self.log.info('Setting up serial handler')
        self.serial = SerialComms(baudrate=115200, timeout= 20)
        self.serial_ports = [str(i) for i in self.serial.ports]
        self.log.debug(f'Available serial ports {self.serial_ports}')
        
        # Main entry point
        self.main()
        
    def print_log(self, buffer):
        '''
        Print and log

        :param buffer: msg to logged and printed
        :type buffer: string, bytes
        '''
        print(buffer)
        self.log.info(buffer)

    def fc_set_mode(self, selected_mode:int):
        '''
        construct a set mode command with the 
        selected_mode

        :param selected_mode: mode to be set
        :type selected_mode: int
        '''
        self.serial.sendData(f'set mode {selected_mode} \n')
        
    def fc_init(self):
        '''
        flight controller init
        '''
        self.serial.connect('COM7')
        
        self.fc_set_mode(self.UNINITIALIZED)
        self.print_log(self.serial.getData())
        
        self.fc_set_mode(self.INITIALIZED)
        done = False
        while not done:
            line = self.serial.getData()
            self.print_log(line)
            if 'INITIALIZATION complete' in str(line):
                done = True

        self.fc_set_mode(self.ACTIVE)
        self.print_log(self.serial.getData())
    
    def main(self):
        '''
        Main entry point
        '''
        self.log.info('Setting up flight controller')
        self.fc_init()
        
        
        
        
if __name__ == '__main__':
    nav = NavigationComputer()
