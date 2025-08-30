## ROS介绍
- 传感器与算法掉包工程
- 话题分模块体系
### 新建功能包与编译
ROS 工作空间
```txt
ros_ws/ #工作空间根目录 colcon build --symlink-install
|-src/ #功能包源码路径 ros2 pkg create 
|  |-package1/
|  |  |-package1_1/
|  |  |  |-CmakeList.txt
|  |  |-package1_2/
|  |  |  |-CmakeList.txt
|  |-package2/
|  |  |-CmakeList.txt
|-build/ #编译路径
|-log/ #日志路径
|-install/ #结构路径
```
### 两种功能包编译工具链与两种构建命令
#### ament_cmake
- c++节点
- launch启动文件+配置文件
- python节点,***不包好***
#### ament_python
- python节点
#### colcon build
- 复制安装
- 更改任何内容都需要重新编译安装
#### colcon build --symlink-install
- 链接安装
- 更改launch文件或者python脚本可以不重新编译
##  ROS 话题
- 最基本的流程图
![alt text](image.png)
- 2025赛季流程图
![alt text](image-1.png)

- 任务分节点,通信靠话题
### ROS话题四要素
#### 话题名称
- 基本没坑
#### 消息类型
- 常用的为传感器消息，tf消息以及字符串消息。
- `不要`自定义消息,有特殊需求通过json+str消息类型实现
#### 服务质量+队列深度
- 巨坑
- 常见配置有`BEST_EFFORT`,`RELIABLE`与`DURABILITY`
- DURABILITY仅在tf_static中使用,不要在其他`任何地方`使用
- 队列深度+服务质量会影响高负载情况下话题数据的发送
- 把队列深度从10改到0会明显改善传感器数据的丢包情况（mid360桥接）
- [详细内容](https://mp.weixin.qq.com/s/J63fO4c_QIseLGQd5W2fAw?poc_token=HFyMsmij6mHtwf00dqkCj7_cnGW45W2UfalGPKcN)
#### 时间戳
- 巨坑,出现在传感器消息与tf等包含时间信息的消息中
- 会影响ros包录制与播放,以及节点的使用
- tf的时间戳可能导致程序卡死

一个简单的发布者与订阅者

```python
from std_msgs.msg import String
class NodeSubscribe(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)

    def command_callback(self,msg):
        speed = 0.0
        if msg.data=="backup":
            speed = -0.2
        self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')


class NodePublisher(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.command_publisher_ = self.create_publisher(String,"command", 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        """
        定时器回调函数
        """
        msg = String()
        msg.data = 'backup'
        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'发布了指令：{msg.data}')    #打印一下发布的数据
```
- 上面的String不是python自带类型,是ros2中的消息类型
## ROS 参数
- 在`源码之外`更改节点的设定值(命令行,launch)
- 参数有的支持运行时更改,有的只允许启动前更改
- 支持在`rqt`中动态显示参数
![alt text](image-2.png)
![alt text](image-3.png)
### 参数的构成
- 键值对,python字典 "key" : value
- 值的范围:int,float,bool,string和对应的列表
### 在节点中的使用
- 先要声明一个参数和默认值
- 用的时候需要取参数
```python
class Communicate_t(Node):
    def __init__(self):
        super().__init__('communicate_t')
        self.declare_parameter('serial_port', '/dev/serial_ch340')
        self.declare_parameter('serial_baudrate', 230400)
        self.serial_port=self.get_parameter('serial_port').value
        self.baudrate=self.get_parameter('serial_baudrate').value
```
- 直接通过命令行传递
```bash
ros2 run <pkg_name> <node_name> param_name:=value
```
### 参数配置文件yaml
- yaml文件头参数较为混乱
- ros2官方的标准为`节点名`+`ros__parameters`
```yaml
usb_cam_node:         //可乱变
  ros__parameters:    //可乱变
    video_device: "/dev/video4"
    framerate: 30.0
```
> 在参数少的情况下建议直接在launch中管理参数
## ROS Launch
- 启动节点,可以生成多个实例
- 给节点传递参数
- launch嵌套