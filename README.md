# ROS_First_Mobile_Robot

- Build a Basic 2-Wheeled Robot with a Camera Sensor and Raspberry Pi
- How to Communicate with the Robot (via WiFi and Ethernet)
- Build a Gazebo Simulation of the 2-Wheeled Robot
- Control the 2-Wheeled Robot (Create the Drivers)
- Navigate with the 2-Wheeled Robot (OpenCV)
- Apply Deep Learning Trainings to our 2-Wheeled Robot (openai_ros package)


## Building the Physical Robot

1. Step by step how to mount the 2 wheeled robot (ngerakit)
2. Setting up the Camera and all the ROS environment
    - Install Ubuntu Mate in RaspberryPi
        * Download the UbuntuMate Version for RaspberryPi. We recommedn the version RaspberryPi ARMv7: [DownloadPage](https://ubuntu-mate.org/download/)
        * Burn that image into your microSD card using gnome-disk-utility. To install it just execute this command in your local computer:
            ```sh
            $ sudo apt-get install gnome-disk-utility
            ```
        * Reboot the RaspberryPi with this new image burned and follow the classical Ubuntu configuration. For that you will need to hook up your raspberry pi to a screen through HDMI, connect a mouse and a keyboard.
    - Setup Camera in RaspberryPi
        * Install the following inside RaspberryPi:
            ```sh
            $ sudo apt-get install libraspberrypi-dev
            $ sudo pip install picamera
            ```
        * Edit the file /boot/config.txt ** so that these two elemenst are **UNcommented:
            > start_x=1, 
            > gpu_mem=128
        * Reboot the RaspberryPi:
            ```sh
            $ sudo reboot
            ```
    - Install ROS in RaspberryPi
        ```sh
        $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
        $ sudo apt-get update
        # We dont need the simulation, but need visualization tools
        $ sudo apt-get install ros-kinetic-desktop
        sudo rosdep init
        $ rosdep update
        $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc 
        $ source ~/.bashrc
        $ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
        $ mkdir -p ~/catkin_ws/src
        $ cd ~/catkin_ws/
        $ catkin_make
        $ sudo apt-get install ros-kinetic-teleop-twist-keyboard
        ```
        Kalau gagal coba cek di WiKi ROS Install
    - Install ROS Related packages for the Camera
        ```sh
        $ sudo apt-get install ros-kinetic-compressed-image-transport
        $ sudo apt-get install ros-kinetic-image-common
        $ sudo apt-get install ros-kinetic-image-view
        $ cd ~/catkin_ws/src
        $ git clone https://github.com/dganbold/raspicam_node.git
        $ cd ~/catkin_ws/
        $ catkin_make --pkg raspicam_node
        ```
        Execute inside the RaspberryPi
        Terminal #1
        ```sh
        $ cd ~/catkin_ws/
        $ source devel/setup.bash
        $ roslaunch raspicam_node camera_module_v2_640x480_30fps.launch
        ```
        Terminal #2
        ```sh
        # View Image
        $ rosservice call /raspicam_node/start_capture

        # View Strem
        $ rosrun image_view image_view image:=/raspicam_node/image_raw
        $ rosrun rqt_image_view rqt_image_view
        ```

        https://www.theconstructsim.com/publish-image-stream-ros-kinetic-raspberry-pi/

        https://www.youtube.com/watch?v=TABVZf5vKVA&list=PLK0b4e05LnzYHQkvmEN4YY2VCB5OdEMVV



## Creating a Simulation of the Robot

1. Creating URDF file
    - Create a new package
        ```sh
        $ roscd; cd ..; cd src; cd ROS_First_Mobile_Robot
        $ catkin_create_pkg rosbots_description rospy
        ```
    - Create urdf folder
        ```sh
        $ roscd rosbots_description
        $ mkdir urdf; cd urdf;
        $ touch rosbots.xacro
        $ chmod +x rosbots.xacro
        ```
    - Copy the .dae files
        ```sh
        $ roscd duckietown_description;
        $ cp -rf meshes ~/catkin_ws/src/ROS_First_Mobile_Robot/rosbots_description/
        ```