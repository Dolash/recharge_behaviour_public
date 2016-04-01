# README #

Hello, and thank you for considering the Recharge Behaviour node for your ROS project! This handy, dandy Create-compatible node work with vicon_bridge and ca_driver to monitor the battery level on your Create and if the level is too low begins sending movement commands to direct the robot back to a given charger location. 

### What is this repository for? ###

* Quick summary - Battery monitor and recharge controller for the Create robot.
* Version 0.1 - the first working one
New in this version: All basic functionality herein described

### How do I get set up? ###

* Summary of setup
Just download this repo onto any computer you intend to use to control the Create, so that could be a laptop plugged in for a temporary test or something more permanent like an Odroid that's also tapping the Create battery. It'll work out of the box, but you need to modify the .launch file with the specifics for your robot.
* Configuration
All the relevant configuration details are in the .launch file, which includes the name (must be the same as the object name given in the Vicon Tracker, as it's used to look itself up over vicon_bridge), the X and Y coordinates of its assigned charger in Vicon space, what the thresholds should be for battery level as well as what times to use for both up-time and down-time if you decide to use time instead of battery level (the battery reports the Create give are not always accurate). You can set whether to use time or battery level there as well.
* Dependencies
* Deployment instructions
First, launch both the vicon_bridge node and the ca_driver node, with the usb-serial connector cable attaching your computer to the base of the Create. Don't forget to sudo chmod 777 /dev/ttyUSB0 after attaching that cable or the ca_driver won't launch. Then run roslaunch for the recharge_behaviour.launch file and hey presto, it works! Then launch any other behaviour nodes you might want to actually control the robot and the recharge node will step in whenever necessary.

As-is this node should work with a different localization source than the vicon so long as it uses the same message type, although looks to subscribe to vicon_bridge specifically at the moment so that would have to be made more general. Also, rather than sending Twist velocity commands straight to the Create we might in the future be sending them to a Twist Mutex which will decide which nodes' commands to prioritize (presumably the recharge node would rank highly there).

### Who do I talk to? ###

* Repo owner or admin - Jack Thomas, jackt@sfu.ca
But realistically you can talk to anyone else involved in the new Create/Chatterbox project at the Autonomy Lab (Sepher, Lingkang, Jacob etc).