# Cobot Remote for AgileX Robotics

## 文件目录
## HTC VIVE追踪器遥操作
### 启动

1. 遥控器上电，确保基站已经上电，启动命令

``` bash
roslaunch remote_bringup arx_remote.launch
```
2. 在rviz中确保追踪器的tf变换不抖动不异常，启动机械臂，开始遥操作