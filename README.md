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
    - Setup Camera in RaspberryPi
    - Install ROS in RaspberryPi
    - Install ROS Related packages for the Camera