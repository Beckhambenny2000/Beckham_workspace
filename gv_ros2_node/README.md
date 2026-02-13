
## gv camera ros2 node

## RUN
```sh
 
ros2 launch gv_ros2 gv_camera.launch.py 

```

## 1.Dependency library

```sh
1. sudo cp ./res/50-myusb.rules /etc/udev/rules.d/
2. sudo udevadm control --reload-rules
3. sudo /etc/init.d/udev restart
4. sudo apt install libusb-1.0-0-dev libuvc-dev
5. sudo apt install libyaml-cpp-dev
```

## 2.Compile

```sh
2.1 cd gv_ros2
2.2 colcon build --packages-select gv_ros2
```

## 3.Running

```sh
3.1  . install/local_setup.sh
3.2  ros2 launch gv_ros2 gv_ros2.launch.py
```

## 4.Config params

```sh
### camera config

# set camera streams status(0 (close), 1(ir), 2(depth), 3(color), 4(ir+depth), 5(color+depth))
stream_status: 5 

# publish pointcloud
publish_pointcloud: true

# depth align to color
align_depth_to_color: false

# set camera bandwidth(0, 512,	1020,	1536,	2040,	2556,	3060)
band_width: 3060

# the max depth distance  of pointcloud
max_distance: 4.0

# show image by opencv
show_image: false

# output stream log
show_data_size: false
```
## 5.Precautions
```sh
5.1 IR data stream and depth data stream cannot be enabled at the same time.

5.2 IR data stream and color data stream cannot be enabled at the same time.
```
