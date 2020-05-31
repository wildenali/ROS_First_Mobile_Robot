# ROS_First_Mobile_Robot

- Build a Basic 2-Wheeled Robot with a Camera Sensor and Raspberry Pi
- How to Communicate with the Robot (via WiFi and Ethernet)
- Build a Gazebo Simulation of the 2-Wheeled Robot
- Control the 2-Wheeled Robot (Create the Drivers)
- Navigate with the 2-Wheeled Robot (OpenCV)
- Apply Deep Learning Trainings to our 2-Wheeled Robot (openai_ros package)


## Building the Physical Robot

1. Step by step how to mount the 2 wheeled robot (ngerakit)
2. Seeting up the Camera and all the ROS environment
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