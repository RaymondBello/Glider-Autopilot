import serial
import random

s = serial.Serial('COM2')



while 1:
    i = b'10,100,20\n'
    
    s.write(i)