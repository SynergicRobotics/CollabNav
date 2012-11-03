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

### How to clean the project?
    rosmake --target=clean CollabNav
or *make distclean*


## Code style
We are going to use [ROS C++ Code Style](http://www.ros.org/wiki/CppStyleGuide).
