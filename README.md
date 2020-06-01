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
2. Settup urdf for RViz
    - Create a launch folder
        ```sh
        $ roscd; cd ..; cd src; cd ROS_First_Mobile_Robot; cd rosbots_description;
        $ mkdir launch; cd launch;
        $ touch rviz.launch
        ```
    - Launch the rviz.launch
        ```sh
        $ roslaunch rosbots_description rviz.launch
        ```
3. Settup for Gazebo
    - Create a launch file
        ```sh
        $ roscd; cd ..; cd src; cd ROS_First_Mobile_Robot; cd rosbots_description; cd launch;
        $ touch spawn.launch
        ```
    - Launch the spawn.launch
        ```sh
        $ roslaunch rosbots_description spawn.launch
        ```
4. Add control to robot (Gazebo)
    - Add code snippet into rosbots.xacro inside the robot tag
        ```sh
        <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <legacyMode>false</legacyMode>
            <alwaysOn>true</alwaysOn>
            <publishWheelTF>true</publishWheelTF>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <updateRate>100.0</updateRate>
            <leftJoint>wheel_left_joint</leftJoint>
            <rightJoint>wheel_right_joint</rightJoint>
            <wheelSeparation>1.1</wheelSeparation>
            <wheelDiameter>0.52</wheelDiameter>
            <wheelAcceleration>1.0</wheelAcceleration>
            <torque>20</torque>
            <commandTopic>/part2_cmr/cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
        </plugin>
        </gazebo>
        ```
    - Empty the simulation
        ```sh
        $ rosservice call /gazebo/delete_model "model_name: 'rosbots'"
        ```
    - Run spawn.launch
        ```sh
        $ roslaunch rosbots_description spawn.launch
        ```
    - Check the part2_cmr/cmd_vel topic
        ```sh
        $ rostopic list
        ```
    - Run the keyboard_teleop to control the robot
        ```sh
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
        ```
    
    > IMPORTANT NOTE!!: In the current Exercise you have just added a Gazebo plugin to your robot, which will allow it to move around. This is pretty cool, but there is one inconvenient. Whenever you delete a model that has a Gazebo plugin, the Gazebo simulation will crash. This is a known issue with Gazebo, but it is quite incovenient for this Unit, since we have been spawning/deleting our robot several times.
    
    > So, from now on, in order to delete the model so that you can spawn an updated one, you will have to change to another Unit (ie, Unit 0) and come back to this one. This way, the Gazbo simulation will be reset and you will be able to spawn again your robot on it.
5. Using the XACRO Macros

    Organize the existing project to make it more readable and modular. Even though our robot description only had a few components, we had written a lengthy xacro file. So, let's start doing some modifications to the files
    - Change the spawn.launch file in launch folder
        from
        ```sh
        <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
        ```
        to
        ```sh
        <param name="robot_description" command="$(find xacro)/xacro.py '$(find rosbots_description)/urdf/rosbots.xacro'" />
        ```
    - Change the rviz.launch file in launch folder
        from
        ```sh
        <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
        ```
        to
        ```sh
        <param name="robot_description" command="$(find xacro)/xacro.py  '$(find rosbots_description)/urdf/rosbots.xacro'"/>
        ```
    - Test the spawn.launch and rviz.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        $ roslaunch rosbots_description rviz.launch
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
        ```
    - Create new xacro file named rosbots.gazebo.xacro inside the URDF folder
        ```sh
        $ roscd; cd ..; cd src; cd ROS_First_Mobile_Robot; cd rosbots_description; cd urdf;
        $ chmod +x rosbots.gazebo.xacro
        ```
    - Edit rosbots.xacro file:
        > Remove <gazebo> tag from 
        Add this line at the beginning of your rosbots.xacro file, inside the <robot> tag.
        > <xacro:include filename="$(find rosbots_description)/urdf/rosbots.gazebo.xacro" />
    - Test the spawn.launch and rviz.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        $ roslaunch rosbots_description rviz.launch
        $ rostopic list , see/part2_cmr/cmd_vel
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
        ```
6. Adding the Sensors to the Robot
