# p3dx_navigation
ROS package navigation to Pioneer P3-DX


- Install rosaria

		$ cd ~/catkin_ws/src
		$ git clone https://github.com/amor-ros-pkg/rosaria.git
		$  cd ..
		$ catkin_make

#

## Real robot
1. Connect the pioneer in USB port, then the laser sick.

2. Open 3 terminal

		$ sudo chmod 777 /dev/ttyUSB*
		$ roslaunch p3dx_navigation pioneer.launch
		$ roslaunch p3dx_navigation p3dx_nav.launch

---

Learn more about using the ROS navigation stack at http://wiki.ros.org/navigation
