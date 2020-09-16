# client

from socket import *

# address = ('127.0.0.1', 12345)
# address = ('192.168.43.108', 1234)
address = ('39.97.120.75', 4321)
s = socket(AF_INET, SOCK_STREAM)
s.connect(address)

data = s.recv(512)
print('the data received is', data)

s.send(bytes('hihi'.encode('utf-8')))

s.close()
