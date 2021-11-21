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
1. Start ros by opening a terminal and running:
	```
	roscore
	```
2. Start the ros node (communication between C++ code and Python code) by running:
	```
	rosrun workstation PupperNode
	```
3. If you don't have a physical ethernet adapter (common on laptops), create a virtual one. This is required to communicate with the keyboard controller interface.
	```
	cd ~/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/src/PythonComms
	bash create_dummy_ethernet
	```
4. Run keyboard controller:
	```
	cd ~/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/src/PythonComms/KeyboardController
	python3 keyboard_joystick.py
	```
5. Start the python code that communicates with the keyboard controller, C++ code, and the pupper. Open a new terminal and enter:
	```
	cd ~/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/src/PythonComms
	python3 run_djipupper.py 
	```
Note: the order of these steps is important.

# Building and uploading the Teensy code
1. Open VS code
2. Install the PlatformIO extension
3. Open the PlatformIO tab and select "Open"
4. Select "Add Existing"
5. Navigate to the folder src/onboard
6. Select Open "onboard"
7. In the PlatformIO tab, select Project Tasks> teensy40 > Build 

# Zeroing the Pupper
The zero position is set with pupper laying on the ground. 
1. Lay pupper flat with elbows and knees back (all feet pointing forward)
2. Rotate hips to raise legs
3. Rotate elbows to raise feet off the ground
4. Rotate hips to lower the legs until the end of the motors touch the ground
5. Rotate elbows to lower feet until they touch the ground (the motors should be rubbing against the ground at this point)
If done correctly, *repeatability is < 1 deg*
