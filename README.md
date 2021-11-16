# PupperWholeBodyControl
Master repo for implementing Whole Body Control on the Stanford Pupper

# Installation
	> git clone git@github.com:makeller1/PupperWholeBodyControl.git

Note: You will need an SSH key on your GitHub profile. See: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

# Setting up Catkin Workspace
The catkin workspace is used to compile the C++ code
1. Create folder for catkin workspace
	e.g. WBC_catkin_ws
3. Add new folder called src
	e.g. WBC_catkin_ws/src
3. In a terminal enter:
	```
	sudo apt update
 	sudo apt install python3-catkin-tools
 	cd  WBC_catkin_ws
 	catkin build
	```
4. Move repo code to  WBC_catkin_ws/src
5. Run one more time:
	```
	catkin build
	```
6. Source the catkin workspace by adding the following line to ~/.bashrc (hidden file in your home directory)
	```bash
	source /home/user/WBC_catkin_ws/devel/setup.bash # Allows ros to find the specific executable inside the package
	```
# Installing gazebo_ros
Assuming you have the ros distro noetic,
```
sudo apt install ros-noetic-gazebo-ros
```
# Building the WBC C++ code
1. Open a terminal and navigate to ~/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/plugins
2. Create a folder called build
	```
	mkdir build
	```
3. Run
	```
	cd build
	cmake ..
	make -j
	```
# Running WBC in simulation
1. Build the WBC C++ code using the steps above
2. Open a terminal and run
	```
	roslaunch workstation load_pupper.launch
	```
# Running WBC on Pupper
1. Start ros by opening a terminal and running
	```
	roscore
	```
2. Start the python code that communicates with the keyboard controller, C++ code, and the pupper. Open a new terminal and enter
	```
	cd ~/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/src
	python3 run_djipupper.py 
	```
	
# Building and uploading the Teensy code
1. Open VS code
2. Install the PlatformIO extension
3. Open the PlatformIO tab and select "Open"
4. Select "Add Existing"
5. Navigate to the folder src/onboard
6. Select Open "onboard"
7. In the PlatformIO tab, select Project Tasks> teensy40 > Build 
