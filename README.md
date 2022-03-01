- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为:
> - LDROBOT LiDAR LD06
> - LDROBOT LiDAR LD19
## 0. 获取雷达的ROS功能包
```bash
$ cd ~

$ mkdir -p ldlidar_ros_ws/src

$ cd ldlidar_ros_ws/src

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
# 或者
$ git clone  https://gitee.com/ldrobotSensorTeam/ldlidar_stl_ros.git
```

## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
$ cd ~/ldlidar_ros_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- 第三步，修改`launch/`目录下雷达产品型号对应的lanuch文件中的`port_name`值，以ld06.launch为例，如下所示.

```xml
<launch>
 <node name="LD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_node" output="screen" >
 <param name="product_name" value="LDLiDAR_LD06"/>
 <param name="topic_name" value="LiDAR/LD06"/>
 <param name="port_name" value ="/dev/ttyUSB0"/>
 <param name="frame_id" value="lidar_frame"/>
 </node>
</launch>
```
## 2. 编译方法

使用catkin编译.

```bash
$ cd ~/ldlidar_ros_ws

$ catkin_make
```
## 3. 运行方法

```bash
source devel/setup.bash
```
- 产品型号为 LDROBOT LiDAR LD06

  ``` bash
  roslaunch ldlidar_stl_ros ld06.launch
  ```
- 产品型号为 LDROBOT LiDAR LD19

  ``` bash
  roslaunch ldlidar_stl_ros ld19.launch
  ```
##   4. 测试

> 代码支持ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic版本下测试，使用rviz可视化。

- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz工具打开readme文件所在目录的rviz文件夹下面的ldlidar.rviz文件
```bash
rosrun rviz rviz
```

| 产品型号:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD06 | lidar_frame  | /LiDAR/LD06 |
| LDROBOT LiDAR LD19 | lidar_frame  | /LiDAR/LD19 |


# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. The product models are :
> - LDROBOT LiDAR LD06
> - LDROBOT LiDAR LD19
## 0. get LiDAR ROS Package
```bash
$ cd ~

$ mkdir -p ldlidar_ros_ws/src

$ cd ldlidar_ros_ws/src

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
# or
$ git clone  https://gitee.com/ldrobotSensorTeam/ldlidar_stl_ros.git
```
## step 1: system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the radar in the system (for example, /dev/ttyUSB0)

  - In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
$ cd ~/ldlidar_ros_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- Modify the `port_name` value in the Lanuch file corresponding to the radar product model under `launch/`, using `ld06.launch` as an example, as shown below.

``` xml
<launch>
 <node name="LD06" pkg="ldlidar_stl_ros" type="ldlidar_stl_ros_node" output="screen" >
 <param name="product_name" value="LDLiDAR_LD06"/>
 <param name="topic_name" value="LiDAR/LD06"/>
 <param name="port_name" value ="/dev/ttyUSB0"/>
 <param name="frame_id" value="lidar_frame"/>
 </node>
</launch>
```
## step 2: build

Run the following command.

```bash
$ cd ~/ldlidar_ros_ws

$ catkin_make
```
## step 3: run

```bash
source devel/setup.bash
```
- The product is LDROBOT LiDAR LD06

  ``` bash
  roslaunch ldlidar_stl_ros ld06.launch
  ```
- The product is LDROBOT LiDAR LD19

  ``` bash
  roslaunch ldlidar_stl_ros ld19.launch
  ```
## step 3: test

> The code was tested under ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic, using rviz visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz tool,open the `ldlidar.rviz` file below the rviz folder of the readme file directory
```bash
rosrun rviz rviz
```

| Product:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD06 | lidar_frame  | /LiDAR/LD06   |
| LDROBOT LiDAR LD19 | lidar_frame  | /LiDAR/LD19   |