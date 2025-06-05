#!/bin/bash

# Allow Docker containers to access X server
xhost +local:docker

# Run the container and execute the ROS2 launch command automatically
docker run --name demo2 \
    --runtime=nvidia --gpus all \
    -e "ACCEPT_EULA=Y" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/bus/usb:/dev/bus/usb:rwm \
    --privileged \
    --rm --network=host \
    demo2:nr \
    bash -c "source /opt/ros/humble/setup.bash && source /dev_ws/install/setup.bash && ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=real robot_ip:=192.168.1.239"

#cd ~/dev_ws
#source install/setup.bash
#ros2 launch xarm5_vision_pick_place launch_xarm5.launch.py mode:=real robot_ip:=192.168.1.239
#    -e ROS_DOMAIN_ID=40 \