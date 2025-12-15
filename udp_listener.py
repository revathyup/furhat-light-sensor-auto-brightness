import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 9999

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Listening on UDP port {}...".format(UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024)
    msg = data.decode("utf-8", errors="ignore").strip()
    print("{}: {}".format(addr, msg))
