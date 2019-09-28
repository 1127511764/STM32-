# 选中目标色块 - By: Tarzan - 周五 4月 5 2019
########################################################
#1、选中目标色块
#2、将色彩格式调换为 LAB Color ， 下拉菜单， 选中即可
#3、记录L， A， B 三者的最大值(MAX) 与 最小值(MIN)
#4、将LAB的取值范围设置为阈值 (threshold)
#5、将阈值传入find_blobs函数中， 根据需求设定像素点与矩形面积等阈值约束
########################################################

import sensor, image, time, struct

from pyb import UART

#山外串口助手波形显示函数
def Send_Wave_Packet(a,b,c):
    text = struct.pack("<bbfffbb",      #格式为俩个字符三个整型
                       0x03,            #帧头1
                       0xFC,            #帧头2
           ################数据#########################
                       float(a),           #数据1
                       float(b),           #数据2
                       float(c),           #数据3
           ############################################
                       0xFC,            #帧尾1
                       0x03)            #帧尾2
    uart.write(text)          #串口发送

#单片机串口通信发送数据函数
def Send_optical_flow_Packet(x,y):
    temp = struct.pack("<bbii",         #格式为俩个字符三个整型
                       0xAA,             #帧头1
                       0xAE,             #帧头2
                       int(x * 10000),  #数据1
                       int(y * 10000))  #数据2
    uart.write(temp)                     #串口发送

#串口三配置
uart = UART(3, 115200, timeout_char=1000)

sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
