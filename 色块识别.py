# 串口的连接方式 - P4-RXD P5-TXD
# 参考网址 - http://book.openmv.cc/MCU/serial1.html
# http://book.myopenmv.com/example/basics/helloworld.html2

import sensor, image, struct, math

from pyb import UART         # 调用单片机的相关模块

from pyb import Pin

import json

#山外串口助手波形显示函数
def Send_Wave_Packet(a,b):
    text = struct.pack("<bbffbb",      #格式为俩个字符三个整型
                       0x03,            #帧头1
                       0xFC,            #帧头2
           ################数据#########################
                       float(a),           #数据1
                       float(b),           #数据2
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

#单片机串口通信发送数据函数
def Send_Point_Packet(x,y):
    temp = struct.pack("<bbii",         #格式为俩个字符三个整型
                       0xAA,             #帧头1
                       0xAE,             #帧头2
                       int(x),          #数据1
                       int(y))          #数据2
    uart.write(temp)                     #串口发送


#串口三配置
uart = UART(3,256000)


pin0 = Pin('P0', Pin.OUT_PP, Pin.PULL_UP)
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_DOWN)


# 设置阈值，通过阈值编辑器来进行处理。在取阈值时，在最小值和最大值之间选取其合适范围。
# 在右侧的图像取范围时，其图像取LAB色彩空间模式。

red_threshold_01 = (0, 100, -128, -16, -21, 40)
# 基本只需更改阈值即可

# 摄像头操作 - 无需更改
sensor.reset()     # 初始化摄像头，消除代码残留的影响
sensor.set_pixformat(sensor.RGB565)   # 设置摄像头的采集图像的模式，参数代表图像使用彩色图像 / GRAYSCALE - 灰色图像
sensor.set_framesize(sensor.QQVGA)   #QQVGA 160-120
sensor.skip_frames(10)   # 跳过开头的图像帧数，参数为帧数

sensor.set_auto_whitebal(False)#关闭白平衡。白平衡是默认开启的，在颜色识别中，需要关闭白平衡。

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

while(True):   # 主循环
    img = sensor.snapshot()   # 将摄像头采集到的图像定义为img

    blobs = img.find_blobs([red_threshold_01])  # 寻找颜色函数，只需更改[orange_threshold_01]的值即可。
    # blobs - 表示着找到的图像的像素范围
    #pin1.value(pin0)

    if blobs:
    #如果找到了目标颜色
        max_blob = find_max(blobs)
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())
        x = max_blob.cx()
        y = max_blob.cy()
        #print(x,y)
        if(not (math.isnan(x)) or (math.isnan(y))):
            #Send_Point_Packet(x,y)
            Send_Wave_Packet(x,y)
    #pin1.value(not pin0)





