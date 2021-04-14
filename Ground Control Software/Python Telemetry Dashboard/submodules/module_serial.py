import serial
import serial.tools.list_ports

class Comms:
    baudrate = ''
    portName = ''
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()

    def __init__(self,baudrate,timeout):
        '''Sets up the serial connection and connects to COM9 @115200 baud'''

        self.baudrate = baudrate
        self.timeout = timeout

    def connect(self, portname):
        '''
        Connect to Serial port

        :param portname: COM Port Name
        :type portname: str
        '''
        self.portname = portname
        
        try:
            self.ser = serial.Serial(self.portname, self.baudrate,timeout=self.timeout)
            return True
        except serial.serialutil.SerialException as error:
            print(f"Could not open {portname}. {error}")
            return False 
        
    def close(self):
        '''close serial connection'''
        if(self.ser.isOpen()):
            self.ser.close()
        else:
            print("Already closed")

    def getData(self):
        '''Get serial data from COM port'''
        try:
            raw_data = self.ser.readline()  # read line (single raw_data) from the serial port
            
            
            return raw_data
        except Exception as error:
            print(error)
            return 0
        
    def sendData(self, buffer):
        
        try:
            self.ser.write(str.encode(buffer))
            
        except Exception as error:
            print(error)
            

    def isOpen(self):
        '''check is serial connection is open'''
        return self.ser.isOpen()