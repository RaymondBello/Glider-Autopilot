import socket

class UDP_Manager:
    def __init__(self):
        self.targetIP = "192.168.0.21"
        self.targetPort = 8080
        self.handler = sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        
    def send_data(self, data):
        '''Send data to targetIP, Return True if sent else Return False'''
        packet_out = str(data).encode('utf-8')
        try:
            self.handler.sendto(packet_out, (self.targetIP, self.targetPort))
            return True
        except Exception as error:
            print(f"[ERROR] : {error}")
        return False
    
    def close_connection(self):
        '''Ends UDP communication, returns true'''
        self.handler.close()
        return True
                

    