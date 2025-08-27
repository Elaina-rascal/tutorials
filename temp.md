# 算法组培训大纲
## 培训内容大纲

### 共9课，每课 1 小时左右，偏速成向而非精讲，需要自学

# Part 1 算法基础

### 第一课 环境配置
- linux 刷机到移动硬盘
- 什么是依赖,静态库与动态库
- linux包管理apt,ppa 与 ros包管理rosdep
- cmake 安装全局包
- 配置vscode 进行开发


### 任务: 在linux中安装Voxel-SLAM
- 禁止使用docker 
- 项目链接 https://github.com/hku-mars/Voxel-SLAM
> 验收时间:3天后
### 第二课 语言基础
- 面向对象的思想
- c++的函数模版
- c++的编译过程
- cmake的使用

### 第三课 ROS基础
- ros介绍
- 新建工作空间与添加功能包
- 话题的订阅发布模型
- launch,传参与yaml
- ros bag 介绍
### 任务: 用c++和python编写ROS2节点+launch
- 用面向对象编程
- C++编写发布者,在键盘按下H的时候输出**任意字符串**到一个话题
- Python编写订阅者,在收到**同名字符串**的时候在控制台打印up70
- 两个节点用一个launch进行启动,并且将上文的任意字符串当成参数传入节点

> 验收时间:2天后
<!-- - 使用ros bag 与foxglove进行调试 -->
### 第四课 docker使用
- docker与image 的区别以及dockerhub是什么
- docker的常用语法
- docker run 与docker compose 启动容器
<!-- - 开发容器的使用 -->
- docker的常见问题
### 任务: 完成ROS2功能包调用


### 第五课 图像处理
- opencv 介绍
- hsv与rgb色彩空间
- aruco 码识别与pnp解算
- yolo识别物块

# Part 2 工程项目开发


### 第六课 工作流介绍
- konsole与开发容器使用
- ros2与ros1 之间的通信
- foxglove+数据包的使用
### 第七课 算法通识
- 设备驱动ros包
- imu, 里程计和对应ros消息
- 激光雷达线雷达,点云与laserscan
- 深度相机 
- 串口介绍与收发数据帧,udev设备管理

### 第八课 机器人定位
- tf树介绍
- slam介绍与其缺点
- slam 的数据源需求
- 数据融合 
### 第九课 团队协作与远程开发
- git的使用
- 代码规范与文档
- 三种远程开发方式
- 代码框架介绍