# CollabNav

## Description
CollabNav is a stack that contains the gmapping package, which provides SLAM capabilities.

### [Wiki](https://github.com/SynergicRobotics/CollabNav/wiki)
  * Weekly [progress](https://github.com/SynergicRobotics/CollabNav/wiki/Progress)
  * Information on [Gmapping](https://github.com/SynergicRobotics/CollabNav/wiki/Gmapping), the technique which we will use for our solution.
  * Summary of the [Rendezvous](https://github.com/SynergicRobotics/CollabNav/wiki/Rendezvous-approach) paper, which describes the approach we will use for map sharing and integration. 


### How to clone the repository?
It will be assume there is a [sandbox](http://www.ros.org/wiki/groovy/Installation/Overlays) directory

    roscd
    cd sandbox
    git clone https://github.com/SynergicRobotics/CollabNav.git

### How to compile?
    rosmake CollabNav

### How to run it?
    1. roscore
    2. rosparam set use_sim_time true
    3. rosrun rviz rviz -d `rospack find collabnav_gmapping`/config/rviz_prepared_for_gmapping.vcg
    4. rosrun collabnav_gmapping slam_gmapping scan:=/scan _odom_frame:=/odom
    5. rosbag play --clock (bag_file_name)

or

    1. roslaunch collabnav_gmapping gmapping_and_rviz.launch
    2. rosbag play --clock (bag_file_name)

### How to clean the project?
    rosmake --target=clean CollabNav
or *make distclean*


## Code style
We are going to use [ROS C++ Code Style](http://www.ros.org/wiki/CppStyleGuide).
