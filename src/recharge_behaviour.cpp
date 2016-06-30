#include <tf/transform_datatypes.h>
#include "recharge_behaviour/recharge_behaviour.h"

/*Jacob's function for extracting the yaw from the Quaternion you get from Vicon. Thanks, Jacob!*/
double getYaw(const geometry_msgs::Quaternion& quat) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}

/*I'm not using this, but in the waypoint driving bit this'd be useful to determine how off-target you are.*/
//TODO replace using <angles/angles.h>
double normalizeAngle(const double& a) {
  double angle = a;
  while (angle < -PI) angle += TWO_PI;
  while (angle > PI) angle += TWO_PI;
  return angle;
}

RechargeBehaviour::RechargeBehaviour(ros::NodeHandle& nh) :
  poseReceived(false),
	chargeReceived(false),
	driving(false),
  nh(nh),
  privNh("~") {
  privNh.param<double>("loop_hz", loopHz, 30);
  privNh.param<double>("yaw", yaw, 0.0);

	/*these parameters control at what battery levels to go recharge and to stop recharging.*/
	privNh.param<double>("high_threshold", highThreshold, 0.70);
	privNh.param<double>("mid_threshold", midThreshold, 0.60);
	privNh.param<double>("low_threshold", lowThreshold, 0.50);

	/*The name parameter is used to identify the particular robot*/
	privNh.param<std::string>("name", robotName, "cb13");

	/*The boundaries of the "charger area" the robot patrols when looking for a station to dock at. Starts at y1 and patrols back and forth to y2*/
	privNh.param<double>("charger_y1", chargerY1, -3.01000);
	privNh.param<double>("charger_y2", chargerY2, 3.4000);

	/*Controls which stage of its patrol the robot is in while moving up and down the y-axis looking for a docking station*/
	privNh.param<bool>("charger_patrol_reset", chargerPatrolReset, false);

	/*The boundary of the "charger area" on the X-axis, which the robot drives to before going up and down the Y to check the different stations*/
	privNh.param<double>("charger_x", chargerX, -2.1800);

	/*these parameters are used to control the recharge times when using time rather than battery charge level to control recharging.*/
	privNh.param<bool>("charge_time", chargeTime, true);
	privNh.param<double>("high_time", highTime, 30);
	privNh.param<double>("mid_time", midTime, 20);
	privNh.param<double>("low_time", lowTime, 10);
	privNh.param<double>("high_charge_time", highChargeTime, 30);
	privNh.param<double>("mid_charge_time", midChargeTime, 20);
	privNh.param<double>("low_charge_time", lowChargeTime, 10);

  	const std::string poseTopic = robotName + "/pose";
	chargeLevel = 0.0;
	buoyPresence = 0;
	chargeLatest = 0;
	cycleStartTime = ros::Time::now();
	backupTime = ros::Time::now();
	debugTime = ros::Time::now();
	backupTimeCheck = false;
  // Convert yaw to radians
 	yaw = yaw * PI / 180.0;
	recharging = false;
	chargeState = 0;

  /*The subscriptions, one for the robot's own pose, one for the battery's current charge level*/
  	ownPoseSub = nh.subscribe(poseTopic, 1, &RechargeBehaviour::ownPoseCallback, this);
	chargeLevelSub = nh.subscribe("battery/charge_ratio", 1, &RechargeBehaviour::chargeLevelCallback, this);
	buoySub = nh.subscribe("ir_omni", 1, &RechargeBehaviour::buoyCallback, this);
  /* The publishers, one to cmd_vel to send movement commands,
   * one to the dock topic when it's time to dock,
   * and one to the undock topic when it's time to undock.
   */
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 30);
	dock_pub = nh.advertise<std_msgs::Empty>("dock", 30);
	undock_pub = nh.advertise<std_msgs::Empty>("undock", 30);

	ROS_INFO("[RECHARGE_BEHAVIOUR] Initialized.");
}

RechargeBehaviour::~RechargeBehaviour() {
 	ROS_INFO("[RECHARGE_BEHAVIOUR] Destroyed.");
}

/*For the pose subscriber, extracting that pose from the message.*/
void RechargeBehaviour::ownPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
  	ownPose = *pose;
  	poseReceived = true;
}

/*For the charge level subscriber, extracting that charge level from the message.*/
void RechargeBehaviour::chargeLevelCallback(const std_msgs::Float32 charge) {
	chargeLatest = charge.data;
  	chargeReceived = true;
}

void RechargeBehaviour::buoyCallback(const std_msgs::UInt16 irReading) {
  	buoyPresence = irReading.data;
  	
}


/*For calculating the angle we want to be turned in order to be facing our target (in this case, the charger)*/
float RechargeBehaviour::getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn)
{
	float result = 0;

	//I'm making this adjustment because I am turning the robot's current position into the origin, so the origin x and y are really currentPositionX/Y - currentPositionX/Y.
	//I'm doing this for my brain's sake.
	float nTargetX = (targetX - currentXCoordinateIn);
	float nTargetY = (targetY - currentYCoordinateIn);


	/*So this calculates, if our robot was at the origin and our target is at the appropriate relative position, what angle should we be turned to face them?*/
	float angbc = atan2((nTargetY), (nTargetX));
	result = angbc;
	result = (result - 1.57595);
	/*A quick fix in the event that this desired angle adjustment takes us "off the edge" of pi*/
	if (result < -3.1518)
	{
		result = (3.1518 - (fabs(result) - 3.1518));
	}
	return result;
}


/*When going to recharge, call this to determine what speed forward/what turn angle you need in order to drive back and forth between the edges of the "charging area"*/
void RechargeBehaviour::approachCharger()
{
	float desiredAngle = 0;
	float turnRate = 0.0;
	float velocity = 0.0;
	/*Since the vicon bridge is giving us quaternions, we'll want to get our own yaw back out of it.*/
	yaw = getYaw(ownPose.transform.rotation);
	/*Now we calculate the yaw we'd want from our current position to be driving toward the recharge station.*/
	if (chargerPatrolReset == false)
	{
		desiredAngle = getDesiredAngle(chargerX, chargerY1, ownPose.transform.translation.x, ownPose.transform.translation.y);
	}
	else
	{
		desiredAngle = getDesiredAngle(chargerX, chargerY2, ownPose.transform.translation.x, ownPose.transform.translation.y);
	}
	



		/*If we're moving and our current yaw is within 0.25 of what we want, keep driving*/
		if ((yaw > (desiredAngle - 0.25)) && (yaw < (desiredAngle + 0.25)) && (driving == true))

		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*However, if we're near the "seam" where ~3.1415 becomes ~-3.1415, we need to be fiddly if our desired angle is on the other side*/
		else if ((((yaw > 2.9) && (desiredAngle < -2.9)) || ((yaw < -2.9) && (desiredAngle > 2.9))) && (driving == true))
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*Now, if we're currently turning and have successfully turned to within 0.15 of what we want, start driving again.*/
		else if ((yaw > (desiredAngle - 0.15)) && (yaw < (desiredAngle + 0.15)) && (driving == false))

		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*And again, if we're within ~0.15 but it's at one of the borders, allow it too.*/
		else if ((((yaw > 3) && (desiredAngle < -3)) || ((yaw < -3) && (desiredAngle > 3))) && (driving == false))

		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*If we get here then the difference between our yaw and desired angle is too great, moving or not, seam or no seam, so start turning*/
		else
		{
			move_cmd.linear.x = 0.0;
			move_cmd.angular.z = 0.2;
			driving = false;
		}


    		cmd_vel_pub.publish(move_cmd);

}

/*The battery level monitor while the robot is active, marks the decrease in battery level until it kills them*/
void RechargeBehaviour::whileActive()
{
	/*Change from high charge to mid charge*/
	if ((chargeLevel < highThreshold && chargeState == 0 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(highTime)) && chargeState == 0 && chargeTime == true))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle STATE CHANGE, 0 to 1, charge level: %f.", chargeLevel);
		chargeState = 1;
	}
	/*Change from mid charge to low charge*/
	else if ((chargeLevel < midThreshold && chargeState == 1 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(midTime)) && chargeState == 1 && chargeTime == true))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle STATE CHANGE, 1 to 2, charge level: %f.", chargeLevel);
		chargeState = 2;
	}
	/*Below low charge threshold, go recharge*/
	else if ((chargeLevel < lowThreshold && chargeState == 2 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(lowTime)) && chargeState == 2 && chargeTime == true))
	{
		if (buoyPresence >= 242 && buoyPresence != 255)
			{
				/*Activate docking demo by sending signal to /dock*/
				ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle DOCK FOUND, charge level: %f.", chargeLevel);
				std_msgs::Empty goDock;
				dock_pub.publish(goDock);
				recharging = true;
				chargerPatrolReset = false;
				cycleStartTime = ros::Time::now();
			}

		/*If you're within range of the given charger location, signal the dock demo and reset the appropriate state variables*/
		else if ((abs(ownPose.transform.translation.y - chargerY2) < 0.1 && chargerPatrolReset == false) || (abs(ownPose.transform.translation.y - chargerY1) < 0.1 && chargerPatrolReset == true)) 
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle REVERSING PATROL DIRECTION, charge level: %f.", chargeLevel);
			chargerPatrolReset = !chargerPatrolReset;
		}
		/*Home in on the charger location*/
		else
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle APPROACHING DOCK, charge level: %f.", chargeLevel);
			approachCharger();
		}
	}
	/*Everything's normal, so just report the battery level.*/
	else if (ros::Time::now() > debugTime + ros::Duration(30))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle charge level: %f", chargeLevel);
		debugTime = ros::Time::now();
	}
}

void RechargeBehaviour::whileRecharging()
{
	/*Notes lowest level of charge*/
	if ((chargeLevel > lowThreshold && chargeState == 2 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(lowChargeTime)) && chargeState == 2 && chargeTime == true))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle STATE CHANGE, 2 to 1, charge level: %f.", chargeLevel);
		chargeState = 1;
	}
	/*Crossing from low to middle charge*/
	else if ((chargeLevel > midThreshold && chargeState == 1 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(midChargeTime)) && chargeState == 1 && chargeTime == true))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle STATE CHANGE, 1 to 0, charge level: %f.", chargeLevel);
		chargeState = 0;
		
	}
	/*Crossing to high charge, time to undock*/
	else if ((chargeLevel > highThreshold && chargeState == 0 && chargeTime == false) || ((ros::Time::now() - cycleStartTime > ros::Duration(highChargeTime)) && chargeState == 0 && chargeTime == true))
	{
		if (backupTimeCheck == false)
		{
			backupTime = ros::Time::now();
			backupTimeCheck = true;
		}
		/*If you are sufficiently far away from the charger position, resume activity.*/
		else if (ros::Time::now() > (backupTime + ros::Duration(15)))
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle UNDOCKING COMPLETE, charge level: %f.", chargeLevel);
			recharging = false;
			backupTimeCheck = false;
			cycleStartTime = ros::Time::now();
			buoyPresence = 0;
		}
		/*Otherwise continue to back up slowly*/
		else
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle UNDOCKING, charge level: %f.", chargeLevel);
			std_msgs::Empty goDock;
			undock_pub.publish(goDock);
			move_cmd.linear.x = -0.1;
			move_cmd.angular.z = 0.0;
			cmd_vel_pub.publish(move_cmd);
		}
	}
	/*Nothing changes, keep recharging and report battery level*/
	else if (ros::Time::now() > debugTime + ros::Duration(30))
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle charge level: %f.", chargeLevel);
		debugTime = ros::Time::now();
	}
}

/*One run of the loop, picks which behaviour to do depending on whether the robot is currently seeking to recharge or not. Only fires if it can hear from its subscriptions*/
void RechargeBehaviour::spinOnce() {
	if (poseReceived && chargeReceived) 
	{
	
		/*Updating the set of power level reports for the last minute*/
		chargeHistory.push_back(chargeLatest);
		if (chargeHistory.size() > loopHz*60)
		{
			chargeHistory.erase(chargeHistory.begin());
		}
		double sum = 0;
		for (int i = 0; i < chargeHistory.size(); i++)
		{
			sum += chargeHistory[i];
		}
  		chargeLevel = sum/chargeHistory.size();

		if (recharging == false)
		{
			whileActive();
		}
		else
		{
			whileRecharging();
		}
	}
	else
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] poseReceived: %d chargeReceived: %d", poseReceived, chargeReceived);
	}
  	ros::spinOnce();
}

/*Gets called by main, runs all the time at the given rate.*/
void RechargeBehaviour::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
    if (!rate.sleep()) {
      ROS_WARN("[RECHARGE_BEHAVIOUR] Loop running slowly.");
    }
  }
}
