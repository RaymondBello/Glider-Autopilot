import serial
import serial.tools.list_ports

class Comms:
    baudrate = ''
    portName = ''
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()

    def __init__(self):
        '''Sets up the serial connection and connects to COM9 @115200 baud'''

        self.baudrate = 115200
        print("Available Ports: ")

        for port in sorted(self.ports):
            # List of ports: https://stackoverflow.com/a/52809180
            print(("{}".format(port)))

        self.portName = "COM9"

        # try:
        #     self.ser = serial.Serial(self.portName, self.baudrate)
        # except serial.serialutil.SerialException:
        #     print("Could not open: ", self.portName)

    def close(self):
        '''close serial connection'''
        if(self.ser.isOpen()):
            self.ser.close()
        else:
            print("Already closed")

    def getData(self):
        '''Get serial data from COM port'''
        raw_data = self.ser.readline()  # read line (single raw_data) from the serial port
        
        data = str(str(raw_data).split("'")[1].split("\\")[0]).split(",")
        return data

    def isOpen(self):
        '''check is serial connection is open'''
        return self.ser.isOpen()