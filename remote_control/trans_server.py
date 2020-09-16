# server

from socket import *

address = ('0.0.0.0', 12345)
s = socket(AF_INET, SOCK_STREAM) # s = socket.socket()
s.bind(address)
s.listen(5)

ss, addr = s.accept()
print('got connected from', addr)

ss.send(bytes('byebye'.encode('utf-8')))
ra = ss.recv(512)
print(ra)

ss.close()
s.close()
