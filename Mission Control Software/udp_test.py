import socket
import time 

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client.bind(("", 64886))

while True:
    packet, addr = client.recvfrom(100)
    clean_packet = str(packet).split("'")[1].split(",")
    print(addr)
    print(clean_packet[:3], clean_packet[3:4], clean_packet[4:])
    # # print(round(float(clean_packet[0]), 2))
    # time.sleep(0.5)
    # client.sendto(b"Test Packet", addr)
    # time.sleep(0.5)
