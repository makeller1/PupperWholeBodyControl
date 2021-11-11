# PupperWholeBodyControl
Master repo for implementing Whole Body Control on the Stanford Pupper

# Installation
	> git clone git@github.com:makeller1/PupperWholeBodyControl.git

Note: You will need an SSH key on your GitHub profile. See: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

# Setting up Catkin Workspace
1. Create folder for catkin workspace
2. Add new folder called src
     i.e. WBC_catkin_ws/src
3. In a terminal enter:
	$ sudo apt update
 	$ sudo apt install python3-catkin-tools
 	$ cd  WBC_catkin_ws
 	$ catkin build
4. Move repo code to  WBC_catkin_ws/src
5. Run one more time:
	$ catkin build
