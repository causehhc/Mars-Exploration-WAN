import time

import pygame
import sys
import cv2
import math
import numpy as np
from numpy import cos, sin, arccos, arctan2
import serial
from socket import *
import _thread

cam_global = None
frame_global = None
thread_flag = None


def cap_thread(haha=1):
    global cam_global
    global frame_global
    global thread_flag
    while True:
        if thread_flag:
            if cam_global is not None:
                ret, frame_global = cam_global.read()
        else:
            time.sleep(1)


def video_update(cam, screen, thrs):
    global frame_global
    screen.fill([0, 0, 0])
    # ret, frame = cam.read()
    frame = frame_global
    if frame is not None:
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = cv2.flip(frame, 0, dst=None)

        edge = cv2.Canny(frame, thrs[0], thrs[1], apertureSize=5)
        vis = frame.copy()
        vis[edge != 0] = (0, 255, 0)

        frame = pygame.surfarray.make_surface(frame)
        screen.blit(frame, (0, 0 + 60))

        vis = pygame.surfarray.make_surface(vis)
        screen.blit(vis, (0, 0 + 60))


def event_listener():
    ans = [[0, [0, 0], 0]]
    for event in pygame.event.get():
        tmp = [0, [0, 0], 0]
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                sys.exit()
            if event.key == pygame.K_p:
                pygame.mouse.set_visible(False)
                pygame.event.set_grab(True)
            if event.key == pygame.K_o:
                pygame.mouse.set_visible(True)
                pygame.event.set_grab(False)

            if event.unicode == '':
                tmp[0] = event.key
                # print('[key down]', ' #', event.key, event.mod)
            else:
                tmp[0] = event.key
                # print('[key down]', ' #', event.unicode, event.key, event.mod)
        if event.type == pygame.MOUSEMOTION:
            tmp[1] = event.rel
            # print('[mouse motion]', ' #', event.pos, event.rel, event.buttons)
        # if event.type == pygame.MOUSEBUTTONUP:
        #     # print('[mouse button up]', ' #', event.pos, event.button)
        if event.type == pygame.MOUSEBUTTONDOWN:
            tmp[2] = event.button
            # print('[mouse button down]', ' #', event.pos, event.button)
        ans.append(tmp)
    return ans


def mouse_move_control(size, gameMouse):
    dx, dy = [0, 0]
    range = 100
    if gameMouse:
        dx, dy = pygame.mouse.get_rel()
        mousepos = pygame.mouse.get_pos()
        if mousepos[0] < range:
            pygame.mouse.set_pos(size[0] - range, mousepos[1])
        if mousepos[0] > size[0] - range:
            pygame.mouse.set_pos(range, mousepos[1])
        if mousepos[1] < range:
            pygame.mouse.set_pos(mousepos[0], size[1] - range)
        if mousepos[1] > size[1] - range:
            pygame.mouse.set_pos(mousepos[0], range)
    if abs(dx) >= (size[0] - 2 * range) / 2:
        dx = 0
    if abs(dy) >= (size[1] - 2 * range) / 2:
        dy = 0
    return dx, dy


def keyboard_fun(keys, exit, gameMouse, thrs):
    exit_b = exit
    gameMouse_b = gameMouse
    thrs_b = thrs
    if keys[pygame.K_ESCAPE]:
        exit_b = False
    if keys[pygame.K_p]:
        gameMouse_b = True
        pygame.mouse.set_visible(False)
        pygame.event.set_grab(True)
    if keys[pygame.K_o]:
        gameMouse_b = False
        pygame.mouse.set_visible(True)
        pygame.event.set_grab(False)
    if keys[pygame.K_u]:
        if thrs[0] < 10000:
            thrs_b[0] = thrs[0] + 100
    if keys[pygame.K_j]:
        if thrs[0] > 0:
            thrs_b[0] = thrs[0] - 100
    if keys[pygame.K_i]:
        if thrs[1] < 10000:
            thrs_b[1] = thrs[1] + 100
    if keys[pygame.K_k]:
        if thrs[1] > 0:
            thrs_b[1] = thrs[1] - 100
    return exit_b, gameMouse_b, thrs_b


def get_key(exit, gameMouse, thrs, power):
    keys = pygame.key.get_pressed()
    # print(keys)
    exit, gameMouse, thrs = keyboard_fun(keys, exit, gameMouse, thrs)
    fwd = keys[pygame.K_s] - keys[pygame.K_w]
    strafe = keys[pygame.K_d] - keys[pygame.K_a]
    roll = keys[pygame.K_e] - keys[pygame.K_q]
    special = keys[pygame.K_c]
    if special:
        special = 1
    else:
        special = 0
    if keys[pygame.K_LSHIFT]:
        if power < 95:
            power += 1
    if keys[pygame.K_LCTRL]:
        if power > 0:
            power -= 1
    if keys[pygame.K_z]:
        power = 95
    if keys[pygame.K_x]:
        power = 0
    move = fwd, strafe, roll, power, special
    return exit, gameMouse, thrs, move, power


def get_mouse(size, gameMouse):
    mouse_wheel = 0
    for event in pygame.event.get():
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4 or event.button == 5:
                mouse_wheel = event.button
    mousekeys = pygame.mouse.get_pressed()
    mousekeys = mousekeys, mouse_wheel
    temp = [[0, 0, 0], 0]
    temp[0][0] = 1 if mousekeys[0][0] else 0
    temp[0][1] = 1 if mousekeys[0][1] else 0
    temp[0][2] = 1 if mousekeys[0][2] else 0
    temp[1] = mousekeys[1]

    # 鼠标移动捕获+处理
    mouse_dx, mouse_dy = mouse_move_control(size, gameMouse)
    # print(mouse_dx, mouse_dy)
    mouse_dx_dy = mouse_dx, mouse_dy
    return temp, mouse_dx_dy


def data_analysis(angle, q0, q1, q2):
    ans_angle = [angle[0] * (180 / math.pi) + 180 - 60, -q0 * (180 / math.pi) + 40,
                 q1 * (180 / math.pi) + 75, (angle[4] - q2) * (180 / math.pi) + 180,
                 90 - angle[1] * (180 / math.pi)]
    ans_angle[0] = 180 - ans_angle[0]
    ans_angle[4] = ans_angle[4] * 4
    ans_angle[4] = 180 - ans_angle[4]
    if ans_angle[4] > 120:
        ans_angle[4] = 120
    if ans_angle[4] < 70:
        ans_angle[4] = 70
    return ans_angle


def inform_update(move, mousekeys, mouse_dx_dy, thrs, screen):
    font = pygame.font.SysFont('arial', 20)
    font1 = pygame.font.SysFont('arial', 15)
    color = 255, 255, 0
    color2 = 255, 0, 0
    color3 = 0, 255, 0

    keyboard = font.render("Key: " + str(move), False, color)
    mouseClick = font.render("Click: " + str(mousekeys), False, color)
    mouseMove = font.render("Move: " + str(mouse_dx_dy), False, color)

    window2 = font1.render("Vehicle_xy", False, color)
    window3 = font1.render("Manipulator_z", False, color)
    window4 = font1.render("Paw", False, color)

    thrs1 = font.render("thrs1: " + str(thrs[0]), False, color3)
    thrs2 = font.render("thrs2: " + str(thrs[1]), False, color3)

    screen.blit(keyboard, (10, 430 + 60 + 60))
    screen.blit(mouseClick, (10, 460 + 60 + 60))
    screen.blit(mouseMove, (10, 490 + 60 + 60))
    screen.blit(thrs1, (10, 10))
    screen.blit(thrs2, (150, 10))

    pygame.draw.line(screen, color2, (213, 420 + 60 + 60), (213, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (356, 420 + 60 + 60), (356, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (499, 420 + 60 + 60), (499, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (0, 60), (640, 60), 2)
    pygame.draw.line(screen, color2, (0, 420 + 60 + 60), (640, 420 + 60 + 60), 2)

    screen.blit(window2, (218, 425 + 60 + 60))
    screen.blit(window3, (361, 425 + 60 + 60))
    screen.blit(window4, (504, 425 + 60 + 60))


def init_inform_update(screen, flag):
    font = pygame.font.SysFont('arial', 20)
    font1 = pygame.font.SysFont('arial', 15)
    font2 = pygame.font.SysFont('arial', 90)

    color = 255, 255, 0
    color2 = 255, 0, 0
    color3 = 0, 255, 0

    keyboard = font.render("Key: " + str(0), False, color)
    mouseClick = font.render("Click: " + str(0), False, color)
    mouseMove = font.render("Move: " + str(0), False, color)

    window2 = font1.render("Vehicle_xy", False, color)
    window3 = font1.render("Manipulator_z", False, color)
    window4 = font1.render("Paw", False, color)

    thrs1 = font.render("thrs1: " + str(0), False, color3)
    thrs2 = font.render("thrs2: " + str(0), False, color3)

    if flag:
        noVideo = font2.render("NO VIDEO", False, color)
        screen.blit(noVideo, (20, 80))
    else:
        noVideo = font2.render("Connecting...", False, color)
        screen.blit(noVideo, (20, 80))

    screen.blit(keyboard, (10, 430 + 60 + 60))
    screen.blit(mouseClick, (10, 460 + 60 + 60))
    screen.blit(mouseMove, (10, 490 + 60 + 60))
    screen.blit(thrs1, (10, 10))
    screen.blit(thrs2, (150, 10))

    pygame.draw.line(screen, color2, (213, 420 + 60 + 60), (213, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (356, 420 + 60 + 60), (356, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (499, 420 + 60 + 60), (499, 520 + 60 + 60), 2)
    pygame.draw.line(screen, color2, (0, 60), (640, 60), 2)
    pygame.draw.line(screen, color2, (0, 420 + 60 + 60), (640, 420 + 60 + 60), 2)

    screen.blit(window2, (218, 425 + 60 + 60))
    screen.blit(window3, (361, 425 + 60 + 60))
    screen.blit(window4, (504, 425 + 60 + 60))


def control_ui(angle, move, mouse_dx_dy, mousekeys, screen):
    """
    angle[0]: 机械臂旋转角度
    angle[1]: 机械臂夹爪状态
    angle[2]: 机械臂上升角度
    angle[3]: 机械臂延展长度
    angle[4]: 机械臂夹爪角度
    angle[5]: 车体旋转角度
    """
    color = 0, 255, 0
    """
    车体机械臂状态
    """
    rotateZoom = 1 / 500
    p = 30
    # if mousekeys[0][1] != 1:  # 鼠标控制
    if move[4] != 1:  # 键盘c控制
        angle[0] += (mouse_dx_dy[0] * rotateZoom)
    # if angle[0] > 2*math.pi:
    #     angle[0] -= 2*math.pi
    # if angle[0] < -2*math.pi:
    #     angle[0] += 2*math.pi
    if angle[0] < -math.pi:
        angle[0] = -math.pi
    if angle[0] > 0:
        angle[0] = 0
    base_x, base_y = 284, 480 + 60 + 60
    base_x_2, base_y_2 = p * math.cos(angle[0]) + base_x, p * math.sin(angle[0]) + base_y
    pygame.draw.line(screen, color, (base_x, base_y), (base_x_2, base_y_2), 2)
    """
    车体状态
    """
    angle[5] = 0
    base_x, base_y = 284, 480 + 60 + 60
    roll_x = 15 * cos(angle[5] + math.pi / 4), 15 * cos(angle[5] + 3 * math.pi / 4), \
             15 * cos(angle[5] - 3 * math.pi / 4), 15 * cos(angle[5] - math.pi / 4)
    roll_y = 25 * sin(angle[5] + math.pi / 4), 25 * sin(angle[5] + 3 * math.pi / 4), \
             25 * sin(angle[5] - 3 * math.pi / 4), 25 * sin(angle[5] - math.pi / 4)
    pointlist = (base_x + roll_x[0], base_y + roll_y[0]), (base_x + roll_x[1], base_y + roll_y[1]), \
                (base_x + roll_x[2], base_y + roll_y[2]), (base_x + roll_x[3], base_y + roll_y[3])
    pygame.draw.polygon(screen, color, pointlist, 2)
    """
    机械臂1状态
    """
    rotateZoom = 1 / 500
    # if mousekeys[0][1] != 1:  # 鼠标控制
    if move[4] != 1:  # 键盘c控制
        angle[2] += (mouse_dx_dy[1] * rotateZoom)
    if angle[2] > 0:
        angle[2] = 0
    if angle[2] < -math.pi / 2:
        angle[2] = -math.pi / 2
    """
    机械臂方向状态
    """
    rotateZoom = 2
    distance = 70
    if mousekeys[1] == 4:
        if angle[3] < distance - rotateZoom:
            angle[3] += rotateZoom
    if mousekeys[1] == 5:
        if angle[3] > rotateZoom:
            angle[3] -= rotateZoom

    base_x, base_y = 400, 500 + 60 + 60
    x, y = angle[3] * math.cos(angle[2]), angle[3] * math.sin(angle[2])
    pygame.draw.line(screen, (255, 255, 255), (base_x, base_y), (base_x + x, base_y + y), 2)
    """
    逆运动学求解要达到(x,y)需要转动的角度，
    返回机器人各关节需要转动的角度
    """
    a0 = 35
    a1 = 35
    q1 = arccos((x ** 2 + y ** 2 - a0 ** 2 - a1 ** 2) / (2 * a0 * a1))
    q0 = arctan2(y, x) - arctan2(a1 * sin(q1), a1 * cos(q1) + a0)
    """
    根据各个关节角计算各个关节的位置.
    注意：所使用的变量与模拟实体对应关系如下所示：
    (joint0)——连杆0——(joint1)——连杆1——[joint2]
    """
    joint0 = np.array([0, 0])
    # 计算关节1的位置
    # q0,q1分别是第0和第1个关节的关节角
    joint1 = joint0 + [a0 * math.cos(q0), a0 * math.sin(q0)]
    # # 计算关节2的位置
    # # 注意：q1是杆1相对于杆0的延长线的转角，而杆0相对水平线的转角是q0
    # # 所以杆1相对水平线的转角是(q0+q1), 而joint2是杆1的末端
    joint2 = joint1 + [a1 * math.cos(q0 + q1), a1 * math.sin(q0 + q1)]
    # 三个关节的坐标
    x = [joint0[0], joint1[0], joint2[0]]
    y = [joint0[1], joint1[1], joint2[1]]
    """
    绘制当前状态下的机械臂
    """
    pygame.draw.line(screen, color, (base_x, base_y), (base_x + x[1], base_y + y[1]), 2)
    pygame.draw.line(screen, color, (base_x + x[1], base_y + y[1]), (base_x + x[2], base_y + y[2]), 2)
    """
    机械臂末端状态
    """
    rotateZoom = 1 / 500
    q2 = q0 + q1
    # if mousekeys[0][1]:  # 鼠标控制
    if move[4]:  # 键盘c控制
        angle[4] += (mouse_dx_dy[1] * rotateZoom)
    if angle[4] > math.pi / 2 + q2:
        angle[4] = math.pi / 2 + q2
    if angle[4] < -math.pi / 2 + q2:
        angle[4] = -math.pi / 2 + q2
    base_x, base_y = base_x + x[2], base_y + y[2]
    p = 10
    base_x_2, base_y_2 = p * math.cos(angle[4]) + base_x, p * math.sin(angle[4]) + base_y
    pygame.draw.line(screen, color, (base_x, base_y), (base_x_2, base_y_2), 2)
    """
    夹爪状态
    """
    rotateZoom = 1 / 50
    p = 30
    if mousekeys[0][0]:
        angle[1] += (1 * rotateZoom)
    if mousekeys[0][2]:
        angle[1] -= (1 * rotateZoom)
    if angle[1] > math.pi / 2:
        angle[1] = math.pi / 2
    if angle[1] < math.pi / 4:
        angle[1] = math.pi / 4
    base_x, base_y = 570, 470 - 15 + 60 + 60
    base_x_2, base_y_2 = p * math.cos(angle[1]) + base_x, p * math.sin(angle[1]) + base_y
    base_x_3, base_y_3 = base_x - p * math.cos(angle[1]), p * math.sin(angle[1]) + base_y
    pygame.draw.line(screen, color, (base_x, base_y), (base_x_2, base_y_2), 2)
    pygame.draw.line(screen, color, (base_x, base_y), (base_x_3, base_y_3), 2)

    return angle, [q0, q1, q2]


def mapAndTrans(a):
    bit = [0, 0]
    fre = int(a * 2000 / 180)
    bit[0] = fre & 0xff
    bit[1] = fre >> 8

    # print(a, fre, bit[0], bit[1])
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
    cmd = [0x7b, 0x00, 0x00, 0x00, 0x7d]

    cmd[2] = move[0] * move[3]
    cmd[1] = move[1] * move[3]
    cmd[3] = move[2] * move[3]
    print(type(cmd[0]), type(cmd[1]))
    print(cmd[1], cmd[2], cmd[3])

    # ser.write(cmd)


def tcpTrans(TCPSock, ans_angle, move):
    list = []
    for i in ans_angle:
        list.append(i)
    for i in move:
        list.append(i)
    str_data = str(list)
    data = str_data.encode("utf-8")
    # print("weatting.....")
    try:
        TCPSock.send(data)
        return False
    except:
        return True


def connect(flag):
    global cam_global
    try:
        # 摄像头选择
        if flag == 0:
            cam_address = "http://39.97.120.75:9000/?action=stream"
        elif flag == 1:
            cam_address = "http://192.168.2.194:8080/?action=stream"
        else:
            cam_address = 0
        cam_global = cv2.VideoCapture(cam_address)

        # tcp连接主机
        if flag == 0:
            host = "39.97.120.75"
            port = 4321
        elif flag == 1:
            host = "192.168.2.194"
            port = 12345
        else:
            host = "127.0.0.1"
            port = 12345
        address = (host, port)
        TCPSock = socket(AF_INET, SOCK_STREAM)
        TCPSock.connect(address)
        return cam_global, TCPSock
    except:
        return None, None


def start_test(screen, size):
    # 开始程序
    global cam_global
    clock = pygame.time.Clock()
    clock.tick(60)
    power = 0
    angle = [-math.pi / 2, math.pi / 4, 0, 1, 0, 0]
    thrs = [10000, 10000]
    gameMouse = False
    cam_global = cv2.VideoCapture(0)
    exit = False
    if cam_global:
        exit = True
    while exit:
        # 事件处理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # 退出窗口
                pygame.quit()
                sys.exit()
        # webcam刷新
        video_update(cam_global, screen, thrs)
        # 键盘按键捕获+处理
        exit, gameMouse, thrs, move, power = get_key(exit, gameMouse, thrs, power)
        # 鼠标捕获+处理
        mousekeys, mouse_dx_dy = get_mouse(size, gameMouse)
        # 仪表刷新+数据回传
        inform_update(move, mousekeys, mouse_dx_dy, thrs, screen)
        angle, [q0, q1, q2] = control_ui(angle, move, mouse_dx_dy, mousekeys, screen)
        # 数据分析
        ans_angle = data_analysis(angle, q0, q1, q2)
        # 数据传输
        # if tcpTrans(TCPSock, ans_angle, move):
        #     exit = False
        # display刷新
        pygame.display.update()
    pygame.mouse.set_visible(True)
    pygame.event.set_grab(False)


def start(screen, size):
    # 开始程序
    clock = pygame.time.Clock()
    clock.tick(60)
    power = 10
    angle = [0.0, 0.7853981633974483, -0.024, 25, 0.034018108424711, 0]
    thrs = [10000, 10000]
    gameMouse = False
    cam_global, TCPSock = connect(1)  # 连接下位机
    exit = False
    if cam_global and TCPSock:
        exit = True
    while exit:
        # 事件处理
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # 退出窗口
                pygame.quit()
                TCPSock.close()
                sys.exit()
        # webcam刷新
        video_update(cam_global, screen, thrs)
        # 键盘按键捕获+处理
        exit, gameMouse, thrs, move, power = get_key(exit, gameMouse, thrs, power)
        # 鼠标捕获+处理
        mousekeys, mouse_dx_dy = get_mouse(size, gameMouse)
        # 仪表刷新+数据回传
        inform_update(move, mousekeys, mouse_dx_dy, thrs, screen)
        angle, [q0, q1, q2] = control_ui(angle, move, mouse_dx_dy, mousekeys, screen)
        # 数据分析
        ans_angle = data_analysis(angle, q0, q1, q2)
        # 数据传输
        if tcpTrans(TCPSock, ans_angle, move):
            exit = False
        # display刷新
        pygame.display.update()
    pygame.mouse.set_visible(True)
    pygame.event.set_grab(False)


def main():
    global thread_flag
    # 各种初始化
    pygame.init()
    pygame.display.set_caption('UI')
    size = 640, 520 + 60 + 60
    screen = pygame.display.set_mode(size)
    _thread.start_new_thread(cap_thread, (1,))

    while True:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # 退出窗口
                pygame.quit()
                sys.exit()
        if keys[pygame.K_RETURN]:
            screen.fill((0, 0, 0))
            init_inform_update(screen, 0)
            pygame.display.update()
            if keys[pygame.K_CAPSLOCK]:
                start_test(screen, size)  # 开始执行(本机)
            else:
                thread_flag = True
                start(screen, size)  # 开始执行
                thread_flag = False

            screen.fill((0, 0, 0))
            pygame.display.update()
        init_inform_update(screen, 1)
        pygame.display.update()


if __name__ == '__main__':
    main()
