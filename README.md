# PX4-ROS-App-Template

Template for developing a containerized application communicating with PX4 via the uXRCE-DDS ROS 2 interface

## Prerequisites

- Docker and Docker Compose
- VS Code/Cursor (optional but recommended) with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

    Note: if you use Cursor you might need to roll back to an earlier version of the Dev Containers extension

## Getting started

0. Get the code

    ```
    git clone git@github.com:freefly-systems/PX4-ROS-App-Template.git
    mv PX4-ROS-App-Template YOUR_APP_NAME
    cd YOUR_APP_NAME
    git submodule update --init --recursive
    ```
    
1. Open VS Code/Cursor and "Reopen in Container" (see pop-up or command palette)

    Note: if you don't use these IDEs, simply run `docker-compose up -d`

2. Verify that things are working by running an example

    ```
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build
    source install/local_setup.bash
    ros2 launch px4_ros_com sensor_combined_listener.launch.py
    ```

2. Begin developing!

    Note: you can enter the terminal of a running container using `docker exec -it CONTAINER_NAME bash`

## More info

See [PX4 docs](https://docs.px4.io/main/en/ros2/user_guide.html#build-ros-2-workspace)