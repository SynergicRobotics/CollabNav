# CollabNav

## Description
CollabNav is a stack that contains the gmapping package, which provides SLAM capabilities.

### How to clone the repository?
It will be assume there is a [sandbox](http://www.ros.org/wiki/groovy/Installation/Overlays) directory

    roscd
    cd sandbox
    git clone https://github.com/SynergicRobotics/CollabNav.git

### How to compile?
    rosmake CollabNav

### How to run it?
    1. roscore
    2. rosrun rviz rviz -d `rospack find collabnav_gmapping`/config/rviz_prepared_for_gmapping.vcg
    3. rosrun collabnav_gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
    4. rosbag play (bag_file_name)

or

    roslaunch collabnav_gmapping gmapping_and_rviz.launch

### How to clean the project?
    rosmake --target=clean CollabNav
or *make distclean*


## Code style
We are going to use [ROS C++ Code Style](http://www.ros.org/wiki/CppStyleGuide).
