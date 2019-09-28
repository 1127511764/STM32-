# ���ڵ����ӷ�ʽ - P4-RXD P5-TXD
# �ο���ַ - http://book.openmv.cc/MCU/serial1.html
# http://book.myopenmv.com/example/basics/helloworld.html2

import sensor, image, time   # ����ͼ����ģ��
from pyb import UART         # ���õ�Ƭ�������ģ��
import json
import time


# ������ֵ��ͨ����ֵ�༭�������д�����ȡ��ֵʱ������Сֵ�����ֵ֮��ѡȡ����ʷ�Χ��
# ���Ҳ��ͼ��ȡ��Χʱ����ͼ��ȡLABɫ�ʿռ�ģʽ��
orange_threshold_01 = (30, 55, -24, 40, -16, -56)# ����ֻ�������ֵ����


# ����ͷ���� - �������
sensor.reset()     # ��ʼ������ͷ���������������Ӱ��
sensor.set_pixformat(sensor.RGB565)   # ��������ͷ�Ĳɼ�ͼ���ģʽ����������ͼ��ʹ�ò�ɫͼ�� / GRAYSCALE - ��ɫͼ��
sensor.set_framesize(sensor.QQVGA)    # ����ͼ�񴰿ڵĴ�С
# VGA 640-480 �� QVGA 320-240 ��QQVGA 160-120
sensor.skip_frames(10)   # ������ͷ��ͼ��֡��������Ϊ֡��

sensor.set_auto_whitebal(False)
#�رհ�ƽ�⡣��ƽ����Ĭ�Ͽ����ģ�����ɫʶ���У���Ҫ�رհ�ƽ�⡣

# clock = time.clock()   # �������������ʱ��ģ�飬�ڱ�������û������

uart = UART(3,115200)   # ��Ƭ���Ĵ������ã��������� - ʹ�ô���3���䲨����Ϊ115200����ֱ�Ӹ���

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

while(True):   # ��ѭ��
    # clock.tick()  # ��ʼ��ʱ��û����
    img = sensor.snapshot()   # ������ͷ�ɼ�����ͼ����Ϊimg

    blobs = img.find_blobs([orange_threshold_01])  # Ѱ����ɫ������ֻ�����[orange_threshold_01]��ֵ���ɡ�
    # blobs - ��ʾ���ҵ���ͼ������ط�Χ

    if blobs:
    #����ҵ���Ŀ����ɫ
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




