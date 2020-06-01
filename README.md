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
        > Remove gazebo tag from 
        Add this line at the beginning of your rosbots.xacro file, inside the robot tag.

        > <xacro:include filename="$(find rosbots_description)/urdf/rosbots.gazebo.xacro" />
    - Test the spawn.launch and rviz.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        $ roslaunch rosbots_description rviz.launch
        $ rostopic list , see/part2_cmr/cmd_vel
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
        ```
6. Adding the Camera to the Robot
    - Edit rosbots.xacro to add camera
    - Reset gazebo
    - Test the spawn.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        ```
    - Edit rosbots.gazebo.xacro to add the camera behavior (like take image or record video) to the link
    - Test the spawn.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        ```
    - Check the topic
        ```sh
        rostopic list
        ```
        should get the following topics now:
        ```sh
        /rosbots/camera1/camera_info
        /rosbots/camera1/image_raw
        /rosbots/camera1/image_raw/compressed
        /rosbots/camera1/image_raw/compressed/parameter_descriptions
        /rosbots/camera1/image_raw/compressed/parameter_updates
        /rosbots/camera1/image_raw/compressedDepth
        /rosbots/camera1/image_raw/compressedDepth/parameter_descriptions
        /rosbots/camera1/image_raw/compressedDepth/parameter_updates
        /rosbots/camera1/image_raw/theora
        /rosbots/camera1/image_raw/theora/parameter_descriptions
        /rosbots/camera1/image_raw/theora/parameter_updates
        /rosbots/camera1/parameter_descriptions
        /rosbots/camera1/parameter_updates
        ```
    - Visualize the camera data with RViz, populate the simulated environment with an object obstacle (object.urdf), to better see the camera's result.
        ```sh
        $ cp /home/simulations/public_sim_ws/src/all/turtlebot/turtlebot_navigation_gazebo/urdf/object.urdf /home/user/catkin_ws/src/ROS_First_Mobile_Robot/rosbots_description
        $ rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/ROS_First_Mobile_Robot/rosbots_description/object.urdf -urdf -x 1 -y 0 -z 1 -model my_object
        $ roslaunch rosbots_description rviz.launch
        ```
        > Add Image

        > Select base_link in the Fixed Frame field.

        > Add two new displays using the Add button on the bottom-left of the RViz screen. The first display should be RobotModel and the other one should be Image.

        > Expand the image display by double-clicking on its name and set the Image Topic to /rosbots/camera1/image_raw (as shown in the image below).
7. Adding the IMU Sensor to the Robot
    - Edit rosbots.gazebo.xacro to add IMU sensor
    - Reset gazebo
    - Test the spawn.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        ```
    - Verify that IMU sensor is actually working correctly
        from
        ```sh
        $ rostopic list
        $ rostopic info /imu/data
        ```
        Should see something like this:
        ```sh
        Type: sensor_msgs/Imu

        Publishers:
            * /gazebo (http://10.8.0.1:43002/)

        Subscribers: None
        ```
        Test the topic
        ```sh
        $ rostopic echo /imu/data -n1
        ```

8. Add some specific data
    - Edit rosbots.gazebo.xacro
    - Test the spawn.launch and rviz.launch file
        from
        ```sh
        $ roslaunch rosbots_description spawn.launch
        $ rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/ROS_First_Mobile_Robot/rosbots_description/object.urdf -urdf -x 1 -y 0 -z 1 -model my_object
        $ roslaunch rosbots_description rviz.launch
        $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
        ```
    - Check Image and IMU data on RViz



## Connecting to Real Robot
### ------------------------
SKIP
### ------------------------



## Creating the Motor Drivers
### ------------------------
SKIP
### ------------------------



## Navigation Line Following
1. Get images from the ROS topic and convert them into OpenCV format
    - Create new package named my_following_line_package inside ROS_First_Mobile_Robot
        ```sh
        $ catkin_create_pkg my_following_line_package rospy
        ```
    - Create 2 new folder named launch and scripts
    - Create python file named line_follower_basics.py in the scripts folder
        > chmod +x line_follower_basics.py
    - Have a look what a different variable inside sensor_msgs/Image
        ```sh
        $ rosmsg show  sensor_msgs/Image
        
        std_msgs/Header header  
            uint32 seq
            time stamp
            string frame_id
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data
        ```
        Extract data from certain variables by doing the following:
        ```sh
        $ rostopic echo -n1 /robot1/camera1/image_raw/height
        $ rostopic echo -n1 /robot1/camera1/image_raw/width
        $ rostopic echo -n1 /robot1/camera1/image_raw/encoding
        $ rostopic echo -n1 /robot1/camera1/image_raw/data
        ```
    - Test the python file
        ```sh
        rosrun my_following_line_package line_follower_basics.py
        ```
2. Process the images using OpenCV libraries to obtain the data we want for the task
- Get Image Info and Crop the image
        ># We get image dimensions and crop the parts of the image we don't need
        
        ># Bear in mind that because its image matrix first value is start and second value is down limit.

        ># Select the limits so that they get the line not too close, not too far, and the minimum portion possible
        
        ># To make the process faster.
        
        >height, width, channels = cv_image.shape
        
        >descentre = 160
        
        >rows_to_watch = 20
        
        >crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
    - Convert from BGR to HSV
        ```sh
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)

        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        ```
    - Apply the mask
    
        Now, you need to generate a version of the cropped image in which you only see two colors: black and white. The white will be all the colors you consider yellow and the rest will be black. It's a binary image.

        Why do you need to do this? It basically has two functions:

        - In doing this, you don't have continuous detection. It is the color or it's NOT, there is no in-between. This is vital for the centroid calculation that will be done after, because it only works on the principal of YES or NO.
        - Second, it will allow the generation of the result image afterwards, in which you extract everything on the image except the color line, seeing only what you are interested in seeing.

        ```sh
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        ```
    - Get The Centroids, draw a circle where the centroid is, and show all the images
        ```sh
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        ```
        show the circle
        ```sh
        cv2.circle(res,(centre_cicle_x, centre_cicle_y), LineWidth,(BGRColour of line),TypeOfLine)
        ```


3. Move the robot along the yellow line, based on the data obtained
    - Get the Proportional control
        ```sh
        error_x = cx - width / 2;
        angular_z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        # Make it start turning
        self.moverosbots_object.move_robot(twist_object)
        ```
    - Create move_robot.py file inside scripts folder (dont forget to get permissiont with chmod +x move_robot.py)
    - Create follow_line_step_hsv.py file inside scripts folder (dont forget to get permissiont with chmod +x follow_line_step_hsv.py)
    - Execute the python file
        ```sh
        $ rosrun my_following_line_package follow_line_step_hsv.py
        ```