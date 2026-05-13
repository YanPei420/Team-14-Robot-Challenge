import socket

UDP_IP = "10.184.236.27"
UDP_PORT = 4210

MESSAGE = b"Stop"

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

sock.sendto(
    MESSAGE,
    (UDP_IP, UDP_PORT)
)