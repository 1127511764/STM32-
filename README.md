# STM32-
硬件：MPU6050模块 HMC5883L磁力计 STM32C8T6单片机 超声波HC-SR04 Mini光流摄像头 OLED显示器

无人机套件(好盈40A电调 F330航模机架 银燕1400KV电机 1045桨叶)

软件：
无人机姿态解算(非四元数解法，不能全姿态) 
磁力计平面校准 
陀螺仪零偏补偿 
串级PID
电调校准
超声波卡尔曼滤波
tft显示屏显示
mini光流与陀螺仪融合(自己去找资料吧 这个光流50块1个定点效果超级好)

实现功能：（前提条件：不超过1.5m的高度） 无人机稳定定高，精度在±5cm以内 
         （前提条件：具有特征信息的地面） 无人机悬停定点 精度约±3cm以内
