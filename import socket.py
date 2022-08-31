#!/usr/bin/env python3
import socket
HOST = "192.168.12.249"
PORT = 30002
s = socket.socket(socket.AF_INIET, socket.socket_STREAM)
s.connect((HOST, PORT))
cmd = "set_digital_out(2,True)" + "\n"
s.send(cmd.encode('utf-8'))
data=s.recv(1024)
s.close()
print("Received",repr(data))