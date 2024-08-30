# Cobot Remote Developer Guide (开发者手册)

## 文件目录

``` bash
├── arm_adapter
├── ESP32
│   ├── esp32_c3
│   ├── esp32_s3
│   ├── espota.py
│   └── ota_remote.bash
├── interface
│   ├── arm_control
│   └── cr5_ros
├── lib
│   ├── libsurvive
│   └── vuer
├── oculus_reader
├── README.md
├── remote_bringup
├── TeleVision
└── vive_ros
```

其中主要包含三大独立的模块，也分别对应目前已知的三种遥操作方法：

### HTC VIVE空间操作单元：

基于HTC VIVE tracker 3.0以及HTC 定位基站2.0，XIAO esp32 S3主控芯片

核心库libsurvive:https://github.com/cntools/libsurvive

```
├── arm_adapter
├── ESP32
├── interface
│   ├── arm_control
│   └── cr5_ros
├── lib
│   ├── libsurvive
├── remote_bringup
└── vive_ros
```

**arm_adapter**: 遥操作核心模块和机械臂交互的适配器，适配各种不同型号的机械臂

**ESP32**: 主控芯片源码，可使用Arduino IDE打开并编译

**interface**: 不同机械臂所使用的接口都放在里面

**lib**: 依赖库，实现追踪器的定位需要依赖libsurvive库

**remote_bringup**: 启动功能包

**vive_ros**: 依赖libsurvive库的ros功能包，负责发布定位器的tf坐标变换，手柄的按键话题

### VR手势遥操作:

基于oculus quest2，3，以及Apple Vision pro

```
├── TeleVision
│   ├── cert.pem
│   ├── img
│   │   └── television.jpg
│   ├── key.pem
│   ├── LICENSE
│   ├── mkcert-v1.4.4-linux-amd64
│   ├── README.md
│   ├── realsense_double_example.py
│   ├── realsense_example.py
│   ├── realsense_pointcloud.py
│   ├── TeleVision.py
│   └── zed_example.py
```

**cert.pem:**自签名证书，可以通过mkcert工具生成

**key.pem:**自签名证书密钥，mkcert工具生成

**mkcert-v1.4.4-linux-amd64:** mkcert工具的二进制文件，可以直接运行

**realsense_double_example.py:** 双realsense在沉浸式空间串流

**realsense_example.py:** 单个realsense相机在沉浸式空间串流

**realsense_pointcloud.py:** 单个相机的rgb点云在沉浸式空间的串流

**TeleVision.py:** 主要功能代码

**zed_example.py:** 原版仓库使用的示例



### VR控制器遥操作

基于meta quest2

```
├── oculus_reader
```

基于开源项目:https://github.com/rail-berkeley/oculus_reader



## HTC VIVE空间操作单元遥操作

### 准备工作

#### 定位基站

<img src="./doc/basestation.png" alt="basestation" style="zoom:50%;" />

固定好两个定位基站，定位基站安装高度最好大于2m小于3m，向下俯视安装10-30度都可，保证在两个基站的视场内即可。官方的定位基站安装教程：https://www.vive.com/cn/support/vive-pro2/category_howto/tips-for-setting-up-base-station2.html

若是第一次使用定位基站，需要手动指定定位基站的频道。使用尖锐物体戳基站背后的按钮，按一次频道加一，范围从1-16，两个基站处于不同的频道即可

一切准备就绪后，定位基站led绿色常亮，代表运行正常

#### 遥操作手柄

<img src="./doc/controller.png" alt="basestation" style="zoom:10%;" />

如果是第一次连接配对手柄，需要设置:

 	1. 主机连接的wifi名称和密码
 	2. 左右手

1. 用usb数据线连接手柄到电脑，电脑弹出u盘选项，代表开始修改配置文件
    <img src="./doc/1.png" alt="basestation" style="zoom:30%;" />

2. 在配置文件中填入wifi名字，密码，以及左右手，还有运行roscore的主机ip，保存并弹出u盘，再次挂载u盘并点开配置文件保存配置

    <img src="./doc/2.png" alt="basestation" style="zoom:30%;" />
    <img src="./doc/3.png" alt="basestation" style="zoom:30%;" />

3. 退出u盘
    <img src="./doc/4.png" alt="basestation" style="zoom:50%;" />

4. 再次挂载u盘，并点开config.txt，此时遥控器显示屏应该显示刚刚修改的wifi字样

	"waiting for WiFi:"

	"TP-LINK"

	<img src="./doc/wifi_2.jpg" alt="basestation" style="zoom:30%;" />

5. 此时退出u盘或拔掉数据线，等待wifi连接成功，显示wifi名，ip地址，左右手

	<img src="./doc/wifi_3.jpg" alt="basestation" style="zoom:30%;" />

修改配置文件需要严格按照上述顺序，目前还不能做到及时修改及时连接，可能是受限于文件系统的写入和读取，或者单片机内部的缓存等原因，导致不能及时修改。该问题待解决

在这个过程中，rgb灯光也可以作为信息指引。

白色常亮：开机初始化中

黄色闪烁：正在连接wifi

蓝色常亮：wifi已经连接，rosmaster未连接

绿色常亮：wifi已连接，rosmaster已连接

#### 定位校准

若是第一次部署代码，需要编译依赖库:

``` bash
roscd remote_bringup
./script/build_survive.sh
```

若是第一次部署定位基站，或者定位基站发生了移动，定位效果不好，都应该进行校准:

```bash
 ./script/calibration_tracker.sh
```

<img src="./doc/cali.png" alt="basestation" style="zoom:80%;" />

启动校准程序后，手持遥控手柄，确保追踪器在两个定位基站的定位范围内

### 启动

1. 遥控器上电，确保基站已经上电，启动命令
- agilex机械臂：
``` bash
roslaunch remote_bringup agilex_remote.launch
```
- arx机械臂：
``` bash
roslaunch remote_bringup arx_remote.launch
```
​	启动代码后，此时代码不发送控制指令，rviz界面中显示追踪器的tf坐标:

<img src="./doc/rviz.png" alt="basestation" style="zoom:50%;" />

2. 在rviz中确保追踪器的tf变换不抖动不异常，启动机械臂驱动，具体视不同机械臂而定
3. 同时长按2个按键，等待约1s后开始发送控制指令
4. 遥操作过程中，同时长按2个按键约1s，遥操作暂停
5. 暂停过程中按下摇杆按键，机械臂回归零点，遥操作结束

### 固件烧录

手柄的esp32使用的是seedstudio的XIAO esp32 s3，官方文档：https://wiki.seeedstudio.com/cn/xiao_esp32s3_getting_started/

有两种方式，一种是直接在Arduino IDE中打开源码编译并烧录，另一种是直接烧录bin二进制文件。两种方式都通过wifi进行远程烧录，usb端口因为复用为存储设备，故无法烧录程序。

Arduino IDE烧录方式：

1. 打开Arduino IDE，根据官方文档搭建开发环境

   <img src="./doc/arduino_setup.png" alt="basestation" style="zoom:40%;" />

2. 点击"文件"->"打开",选择路径打开文件: cobot_remote/ESP32/esp32_s3/esp32_s3.ino，可以看到项目中的所有文件

3. 选择下载端口，如果主机和手柄连接在同一个局域网下，会出现远程端口，有时手柄刚连上wifi，端口显示会有延迟，可以尝试等待或者重启下手柄

<img src="./doc/ide.png" alt="ide" style="zoom:40%;" />

4. 若是第一次选择端口，需要在弹出来的窗口中指定端口所对应的烧录驱动，我们选择“XIAO_ESP32S3”

<img src="./doc/select_xiao.png" alt="select_xiao" style="zoom:40%;" />

5. 在Arduino IDE主页点击“上传”进行烧录，输出终端中显示100%，烧录完成

二进制烧录：

在实际OTA中，我们不想暴露源码给客户，所以使用二进制文件来更新固件

点击“项目”->“导出已编辑的二进制文件”，在项目文件夹路径下出现“build”文件夹，其中包含.bin文件，即是我们所需要的二进制文件

<img src="./doc/download.jpeg" alt="select_xiao" style="zoom:60%;" />

在ESP32文件夹中，运行远程烧录脚本

```bash
python3 espota.py  -d -i 192.168.0.139 -p 3232 -r true -f esp32_s3.ino.bin
```

<img src="./doc/espota.png" alt="select_xiao" style="zoom:50%;" />

uploading进度达到100%，代表烧录完成

参数说明：

- -i: 遥操作端的ip地址，手柄连接wifi后会在屏幕打印ip
- -p: 遥操作端网络下载的端口，固定为3232
- -f: 待烧录的二进制文件的路径

### 设计模式

遥操作代码分为两部分，esp32端和主机端，两部分各司其职，通过ros话题交互。同时vive追踪器是独立的部分，它直接通过无线模块和主机端进行通信

#### esp32端

遥操作端的资料，包括单片机资料和手柄板子原理图，都在项目文件夹中

esp32端主要有以下职责：

1. 读取摇杆的x，y轴ADC数值以及各个按键，霍尔传感器电压
2. lcd屏幕，rgb灯的信息反馈
3. 将信息通过ros转发

代码架构：

esp32_s3.ino作为Arduino IDE的项目文件，存放着"setup"函数和“loop”函数，setup中存放所有的外围设备初始化程序，loop则存放着单片机循环运行的代码。

setup中初始化代码详见代码内注释

loop主函数：

loop主函数最外围不断监测wifi连接状态，如果wifi失去连接，则开始不断重连。如果已经连接，则执行程序主要的功能：

	1. 看门狗刷新，这是ros连接的看门狗，通过主机端的ros心跳包来确保当前连接的稳定，如果断连，后续rgb灯光会改变颜色。同时马达连续快速震动两次代表wifi连接成功
	1. 每40ms运行一次主程序，循环中包括rgb灯光的变化，lcd屏的状态显示，电池电量显示，adc采样和按键的io口监测。提取所有信息后，将信息打包为json格式并序列化，通过ros发布者发布到局域网内，一个循环结束。

除此之外，其他对应设备的相关代码都被放在了各自的头文件中，在每个头文件的开头有详细的功能说明

#### 主机端



## VR远程手势遥操作:

项目主要基于开源工作的早期版本开发:

https://github.com/OpenTeleVision/TeleVision

该项目实际有更新和改进，本项目暂未跟进，依旧使用的最初始版本