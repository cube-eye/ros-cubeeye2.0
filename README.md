# CUBE EYE camera ROS driver (S100D / S110D / S111D / I200D device)
These are packages for using S100D / S110D device with ROS
- #### S100D / S110D / S111D / I200D

Please refer to our website for detailed product specifications. [http://www.cube-eye.co.kr](http://www.cube-eye.co.kr)



## Installation Instructions

The following instructions are written for ROS Noetic, on Ubuntu 20.04

### Step 1 : Install the ROS distribution
- #### Install ROS Noetic, on Ubuntu 20.04

### Step 2 : Install driver
- #### Create a catkin workspace
```bash
$ source /opt/ros/noetic/setup.bash
$ mkdir –p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
Copy the driver source to the path(catkin_ws/src)
```

- #### driver build
```bash
$ catkin_init_workspace
$ cd ..
$ catkin_make clean
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Usage Instructions

Connect the camera power and execute the following command

```bash
$ source install/setup.bash
```

```bash
$ roscore
```

```bash
$ roslaunch cubeeye_camera cubeeye_camera.launch
```

#### Services
- #### Scan CUBE EYE camera
```bash
$ rosservice call /cubeeye_camera_node/scan
```

- #### Connect camera source (camera index)
Connect the first camera (index: 0) in scanned sources.
```bash
$ rosservice call /cubeeye_camera_node/connect 0
```

- #### Run camera (frame type)
Run Amplitude and Depth (type: 6):
```bash
$ rosservice call /cubeeye_camera_node/run 6
```
Run PointCloud (type: 32):
```bash
$ rosservice call /cubeeye_camera_node/run 32
```

- #### Stop camera
```bash
$ rosservice call /cubeeye_camera_node/stop
```

- #### Disconnect camera
```bash
$ rosservice call /cubeeye_camera_node/disconnect
```

#### Topics
Topics are published when a camera is connected. Once a camera starts with a frame type, the frame is populated.
- /cubeeye/camera/depth : Depth Image
- /cubeeye/camera/amplitude : Amplitude Image
- /cubeeye/camera/rgb : RGB Image
- /cubeeye/camera/points : PointCloud Image

#### Operating Test
```bash
$ rqt
/cubeeye/camera/amplitude, /cubeeye/camera/depth
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545764-ca58ce00-55f8-11eb-86a9-3bc091a1aeb9.png"/></p>

```bash
$ rosrun rviz rviz
Fixed Frame : pcl
PointCloud2 : /cubeeye/camera/points
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/104545815-de043480-55f8-11eb-8293-baa2edba664f.png"/></p>

#### Using Dynamic Reconfigure Params
```bash
$rosrun rqt_reconfigure rqt_reconfigure
```
![dynamic_reconfigure](https://user-images.githubusercontent.com/90016619/131959624-b3ff03e2-2eb7-4cc6-bc5d-df39dc74ed89.png)
