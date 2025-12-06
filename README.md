# 梯队任务 by consefact
## 写在前面
本来想尽量每个模块尽量都顾及到一点，但各种各样的环境实在是太折磨人了，加上本身水平一般，稍显简陋，还请见谅

## 启动
roslaunch sim_task task.launch
roslaunch trying trying.launch
roslaunch darknet_ros darknet_ros.launch

## 主体任务

### 1 起飞
template复制粘贴，修改了悬停时间
### 2 避障
把px4_com里的collision_avoidance.cpp改成了.h ,并把px4-command用mavros::PositionTarget 下的位置与速度控制改写，部分参数放到了yaml里，其余基本复制粘贴
又尝试加了x方向过近时y方向加速，以缓解目标，障碍，无人机共线卡死的情况

### 3 穿门
首先 没有完成
有想把避障代码复用，但效果不太好
也有想实现用激光雷达实现门(直线)的识别，这个只停留在想的阶段(没时间)

### 4 穿环
//首先 没有完成
有想过对框的两边识别并从中间穿，停留在想
在知道能够定义前视摄像头时，想过可以用yolo识别方/圆，但不知道有没有现成的合适的数据集，作罢



### 5 目标识别
采用darknet_ros, yolov3-tiny, 完成了对二维码的识别
数据集是网上现成的(带标注)，在物理机win上搭建darknet环境并训练模型
为了完成这个部分，添加以下功能
1. template.h中添加control_by_vel，用于x y方向的定速飞行，有用于单方向飞行时的偏移纠正
  写这个是因为mission_pos_cruise的定点在提速时会改变无人机姿态，总是飞到奇怪的地方，且不利于下视物体识别
2. template.h中添加travers_region，用于遍历目标区域，
3. darknet_control.h中objeion_detect判断是否发现目标
4. darknet_control.h中move_to_center用于移动到bbox中心

因为想实现显卡直通未果，我在测试时使用cpu跑yolo
且修改了下视摄像分辨率，如需修改，yaml里的两个参数image_height/width也请一并修改


######  
> 本来是想完成三种(数字，字母，二维码)的，但现成的数据集有标注格式，每种的类别名等等问题，现成的/AI写的py脚本也不能完全看懂，就先搁置。
但代码本身应该是支持多种类别的。

### 6 颜色识别
问AI得到了大致方法
把ros图像转为openCV可识别，然后在HSV下匹配颜色
但是没时间做了
### 7 降落
template.h里的precision_land在降到一个高度后就会悬停并退出程序，而后被接管，又会上升到一定高度，并没有完成降落
所以改成先降到一定高度，再换用AUTO.LAND降落