# avt_vimba_camera

This repo contains a ROS2 driver for cameras manufactured by [Allied Vision Technologies](https://www.alliedvision.com).
The driver relies on libraries provided by AVT as part of their [Vimba SDK](https://www.alliedvision.com/en/products/software.html).  
This driver was ported from the ROS(1) driver at [AutonomouStuff GitHub](https://github.com/astuff/avt_vimba_camera).

## Building the driver

Driver can be built by ensuring the [ROS2 image_common](https://github.com/ros-perception/image_common/tree/ros2) repo is installed, 
cloning this repository, then installing the Vimba SDK, and editing the CMakeLists.txt file to point to the SDK installation
location (default: ./Vimba_4_1).  
Then source your ROS2 installation and build as usual.  
For example:
```
mkdir cam_drv; cd cam_drv
git clone https://github.com/<this-repo>/avt_vimba_camera.git
cd avt_vimba_camera
(copy the Vimba SDK to ./Vimba_4_1, or edit CMakeLists.txt to point to SDK location)
source (ros2 installation)
colcon build --symlink-install
```

The driver can then be run as a ros2 package/executable, with support for pre-loaded configuration parameters.  
Such as:
```
ros2 run avt_vimba_camera mono_camera_node --ros-args --params-file ./config/mono_c1.yaml
```

