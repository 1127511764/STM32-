# 串口的连接方式 - P4-RXD P5-TXD
# 参考网址 - http://book.openmv.cc/MCU/serial1.html
# http://book.myopenmv.com/example/basics/helloworld.html2

import sensor, image, time   # 调用图像处理模块
from pyb import UART         # 调用单片机的相关模块
import json
import time


# 设置阈值，通过阈值编辑器来进行处理。在取阈值时，在最小值和最大值之间选取其合适范围。
# 在右侧的图像取范围时，其图像取LAB色彩空间模式。
orange_threshold_01 = (30, 55, -24, 40, -16, -56)# 基本只需更改阈值即可


# 摄像头操作 - 无需更改
sensor.reset()     # 初始化摄像头，消除代码残留的影响
sensor.set_pixformat(sensor.RGB565)   # 设置摄像头的采集图像的模式，参数代表图像使用彩色图像 / GRAYSCALE - 灰色图像
sensor.set_framesize(sensor.QQVGA)    # 设置图像窗口的大小
# VGA 640-480 ； QVGA 320-240 ；QQVGA 160-120
sensor.skip_frames(10)   # 跳过开头的图像帧数，参数为帧数

sensor.set_auto_whitebal(False)
#关闭白平衡。白平衡是默认开启的，在颜色识别中，需要关闭白平衡。

# clock = time.clock()   # 定义变量，调用时间模块，在本程序中没有作用

uart = UART(3,115200)   # 单片机的串口设置，参数代表 - 使用串口3，其波特率为115200，可直接更改

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

while(True):   # 主循环
    # clock.tick()  # 开始计时，没作用
    img = sensor.snapshot()   # 将摄像头采集到的图像定义为img

    blobs = img.find_blobs([orange_threshold_01])  # 寻找颜色函数，只需更改[orange_threshold_01]的值即可。
    # blobs - 表示着找到的图像的像素范围

    if blobs:
    #如果找到了目标颜色
        max_blob=find_max(blobs)
        img.draw_rectangle(max_blob.rect())
        img.draw_cross(max_blob.cx(), max_blob.cy())
        x = max_blob.cx()
        y = max_blob.cy()
        print(x)
        uart.write(chr(x))
        time.sleep(10)
'''
else:
        uart.write(chr(1))
        print(0)
'''




