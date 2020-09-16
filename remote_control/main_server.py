import time
from socket import *
import serial


def mapAndTrans(a):
    bit = [0, 0]
    fre = int(a * 2000 / 180)
    bit[0] = fre & 0xff
    bit[1] = fre >> 8
    return bit


def serialTrans_arm(ans_angle, ser):
    cmd = [0x55, 0x55,
           0x14, 0x03,
           0x05, 0x64, 0x00,
           0x00, 0x00, 0x00,
           0x01, 0x00, 0x00,
           0x02, 0x00, 0x00,
           0x03, 0x00, 0x00,
           0x04, 0x00, 0x00]
    bit0 = mapAndTrans(ans_angle[0])
    bit1 = mapAndTrans(ans_angle[1])
    bit2 = mapAndTrans(ans_angle[2])
    bit3 = mapAndTrans(ans_angle[3])
    bit4 = mapAndTrans(ans_angle[4])

    cmd[8] = bit0[0]
    cmd[9] = bit0[1]
    cmd[11] = bit1[0]
    cmd[12] = bit1[1]
    cmd[14] = bit2[0]
    cmd[15] = bit2[1]
    cmd[17] = bit3[0]
    cmd[18] = bit3[1]

    cmd[20] = bit4[0]
    cmd[21] = bit4[1]

    ser.write(cmd)


def serialTrans_car(move, ser):
    cmd = [0xf0, 0x64, 0x64, 0x64, 0xfa]
    cmd[2] = move[0] * move[3]+100
    cmd[1] = move[1] * move[3]+100
    cmd[3] = move[2] * move[3]+100
    # print(cmd)
    ser.write(cmd)
    time.sleep(0.1)


def strspilt(str_data):
    strlist = []
    if str_data[0] != '[':
        return None
    for ch in str_data:
        strlist.append(ch)
        if ch == ']':
            break
    new_str = ''.join(strlist)
    return new_str


def dataDecode(str_data):
    # str_data = "[150.0, 139.18146102102702, 253.3629220420538, 0.8185389789732085, 180.0, 0, 0, 0, 0, 0]"
    str_list = []
    angle = [0.0, 0.0, 0.0, 0.0, 0.0]
    move = [0, 0, 0, 0, 0]

    i = 0
    str_buff = []
    for ch in str_data:
        if ch == ',' or ch == ']':
            str_list.append(''.join(str_buff))
            str_buff.clear()
        elif i > 0:
            str_buff.append(ch)
        i += 1

    i = 0
    for data in str_list:
        if i < 5:
            angle[i] = float(data)
        else:
            move[i - 5] = int(data)
        i += 1
    # print(angle, move)
    return angle, move


def main_server(flag):
    ser_arm = 0
    ser_car = 0
    if flag:
        ser_arm = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
        ser_car = serial.Serial("/dev/ttyUSB1", 9600, timeout=0.5)
    address = ("0.0.0.0", 12345)  # 允许所有域名访问
    TCPSock = socket(AF_INET, SOCK_STREAM)
    TCPSock.bind(address)
    TCPSock.listen(5)
    flag_all = 1
    while flag_all:
        print("watting...")
        tcp, addr = TCPSock.accept()
        print("got connected from", addr)
        flag = 1
        while flag:
            try:
                data = tcp.recv(512)
                str_data = str(data, encoding='utf-8')
                str_data_new = strspilt(str_data)
                if str_data_new is not None:
                    [ans_angle, move] = dataDecode(str_data_new)
                    print(ans_angle, move)
                    if flag:
                        serialTrans_arm(ans_angle, ser_arm)
                        serialTrans_car(move, ser_car)
                else:
                    print("trans error!")
            except:
                print("loss target!")
                flag = 0
                tcp.close()
    TCPSock.close()


if __name__ == '__main__':
    print("start")
    main_server(1)
