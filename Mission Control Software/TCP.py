import websocket
import time

class TCP_Manager:
    def __init__(self):
        self.targetIP = "192.168.0.37"
        self.handler = websocket.WebSocket()
        try:
            self.handler.connect(f"ws://{self.targetIP}")
            print(f"Connected to IP: {self.targetIP}")
        except Exception as error:
            print(error)
    
    def send_data(self, data_type, data):
        '''Send data to tagertIP, return True if sent else Return False'''
        packet_out = (str(data_type) + ":" + str(data))

        try:
            self.handler.send(packet_out)
            return True
        except Exception as error:
            print(error)
        return False

    def receive_data(self):
        '''Receive packet from targetIP return the received packet as an array'''
        packet_in = self.handler.recv()

        return packet_in.split(",")
    
    def close_connection(self):
        '''End communication with client'''
        self.handler.close()
        return True



# TCP = TCP_Manager()

# i = 0
# start_time = time.perf_counter()
# while i < 100:
#     TCP.send_data("Command", "Instruction")
#     packet = TCP.receive_data()
#     print(packet)
    
#     i+=1

# end = time.perf_counter()
# time_per_hadshake = (end - start_time) / 100
# print(f"Time per handshake: {time_per_hadshake} seconds")

# TCP.handler.close()
