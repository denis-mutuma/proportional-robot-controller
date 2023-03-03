# Proportional Robot Controller with ROS

# About the project

This project seeks to move the [turtlesim](http://wiki.ros.org/turtlesim) to a desired destination defined by `x` and `y` cordinates. The objective of the controller is to minimize the error between the current robot position and the desired position. This is achieved by...

In order to move to the desired location, the turtlesim will first turn in the direction of the desired destination and then move towards that point. This is achieved using a proportional ($K_p$) controller.

Internally, the robot controller publishes the robot position at a frequency of 10 $Hz$ and subsribes to the same topic to get the new position.

## Built with:
 - ROS Noetic
 - Catkin
 - C++

# Getting Started

## Prerequisites  

1. Install ROS Noetic from [http://wiki.ros.org/noetic/Installation](http://wiki.ros.org/noetic/Installation)

## Installation
1. Create a catkin workspace
2. Clone the repo

```
git clone https://github.com/denis-mutuma/proportional-robot-controller.git
```

3. run `catkin_make` in the catkin workspace to build the project
4. install tutlesim by running `sudo apt-get install ros-$(rosversion -d)-turtlesim`

# Usage

1. In a new terminal run `roscore` to start the master
2. In a new terminal run turtlesim using: `rosrun turtlesim turtlesim_node`
3. A new window similar to the one below will be opened
![turtlesim1](./images/turtle1.png)
4. In a new terminal run `rosrun practical goto_xy` ro run the proportional robot contoller and you'll be prompted to provide the x and y cordinates via the terminal. You should see the turtle turning (if need be) and moving to the desired location and terminal will show something similar to this:
![output](./images/output.png)
5. The turtle will move to  the disired position and doing this multiple times will produce something similar the image below:
![turtlesim2](./images/turtle2.png)

# Contributing

1. Fork the Project
2. Create your Feature Branch (git checkout -b feature/AmazingFeature)
3. Commit your Changes (git commit -m 'Add some AmazingFeature')
4. Push to the Branch (git push origin feature/AmazingFeature)
5. Open a Pull Request

# License

Distributed under the BSD License.
