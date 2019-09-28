import sensor, image, time, struct, math

from pyb import UART

uart = UART(3, 115200, timeout_char=1000)

def send_optical_flow_packet(x,y,c):
    temp = struct.pack("<bbiii",         #格式为俩个字符三个整型
                       0xAA,             #帧头1
                       0xAE,             #帧头2
                       int(x * 100000),  #数据1
                       int(y * 100000),  #数据2
                       int(c * 100000))  #数据3

#    print(x,y,c)
    uart.write(temp)                     #串口发送

#山外串口助手波形显示函数
def Send_Wave_packet(a,b):
    text = struct.pack("<bbffbb",         #格式为俩个字符三个整型
                       0x03,             #帧头1
                       0xFC,             #帧头2
           ################数据#########################
                       float(a),           #数据1
                       float(b),           #数据2
           ############################################
                       0xFC,             #帧尾1
                       0x03)             #帧尾2
    uart.write(text)                     #串口发送


#判断变量类型的函数
def typeof(variate):
    type=None
    if isinstance(variate,int):
        type = "int"
    elif isinstance(variate,str):
        type = "str"
    elif isinstance(variate,float):
        type = "float"
    elif isinstance(variate,list):
        type = "list"
    elif isinstance(variate,tuple):
        type = "tuple"
    elif isinstance(variate,dict):
        type = "dict"
    elif isinstance(variate,set):
        type = "set"
    return type

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.B64X32) # Set frame size to 64x64... (or 64x32)...
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
extra_fb.replace(sensor.snapshot())

Delta_x = 0
Delta_y = 0

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    # Offset results are noisy without filtering so we drop some accuracy.
    sub_pixel_x = int(displacement.x_translation() * 5) / 5.0
    sub_pixel_y = int(displacement.y_translation() * 5) / 5.0
    response = displacement.response()

    Delta_x = sub_pixel_x + Delta_x
    Delta_y = sub_pixel_y + Delta_y

    #Send_Wave_packet(sub_pixel_x, sub_pixel_y)
    #print(sub_pixel_x)

    if (not (math.isnan(sub_pixel_x)) or (math.isnan(sub_pixel_y)) or (math.isnan(response)) or (math.isnan(Delta_x)) or (math.isnan(Delta_y))):
        send_optical_flow_packet(sub_pixel_x,sub_pixel_y,response)
