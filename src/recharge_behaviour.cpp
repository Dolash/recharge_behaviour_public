#include <tf/transform_datatypes.h>
#include "recharge_behaviour/recharge_behaviour.h"

double getYaw(const geometry_msgs::Quaternion& quat) {
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}

//TODO replace using <angles/angles.h>
double normalizeAngle(const double& a) {
  double angle = a;
  while (angle < -PI) angle += TWO_PI;
  while (angle > PI) angle += TWO_PI;
  return angle;
}

RechargeBehaviour::RechargeBehaviour(ros::NodeHandle& nh_) :
  poseReceived(false),
	chargeReceived(false),
	driving(false);
  nh(nh_) {
  nh.param<double>("loop_hz", loopHz, 60);
  nh.param<double>("distance_x", distX, 2.0);
  nh.param<double>("distance_y", distY, 0.0);
  nh.param<double>("distance_z", distZ, 0.0);
  nh.param<double>("yaw", yaw, 0.0);

	nh.param<double>("charge_level", chargeLevel, 0.0);
	nh.param<double>("high_threshold", highThreshold, 0.70);
	nh.param<double>("mid_threshold", midThreshold, 0.60);
	nh.param<double>("low_threshold", lowThreshold, 0.50);
	nh.param<bool>("recharging", recharging, false);
	nh.param<int>("charge_state", chargeState, 0);
	
	nh.param<double>("charger_x", chargerX, -1.3000);
	nh.param<double>("charger_y", chargerY, 0.6000);


  // Convert yaw to radians
  yaw = yaw * 3.14159 / 180.0;


  ownPoseSub = nh.subscribe(std::string("localization"), 1, &RechargeBehaviour::ownPoseCallback, this);
	chargeLevelSub = nh.subscribe(std::string("battery/charge_ratio"), 1, &RechargeBehaviour::chargeLevelCallback, this);

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", Twist, queue_size=10);
	dock_pub = nh.advertise<std_msgs::Empty>("dock", Empty, queue_size=10);
	undock_pub = nh.advertise<std_msgs::Empty>("undock", Empty, queue_size=10);
  //posePub = nh.advertise<geometry_msgs::Pose>(std::string("goal_pose"), 1);
  ROS_INFO("[RECHARGE_BEHAVIOUR] Initialized.");
}

RechargeBehaviour::~RechargeBehaviour() {
  ROS_INFO("[RECHARGE_BEHAVIOUR] Destroyed.");
}

void RechargeBehaviour::ownPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose) {
  ownPose = *pose;
  poseReceived = true;
}

void RechargeBehaviour::chargeLevelCallback(const float charge) {
  chargeLevel = float;
  chargeReceived = true;
}


/*For calculating the angle we want to be turned in order to be facing our target (in this case, the charger)*/
float RechargeBehaviour::getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn)
{
	float result = 0;
	float originX = 0.0;
	float originY = 0.0;

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


/*When going to recharge, call this to determine what speed forward/what turn angle you need in order to home in on the given charger location*/
void RechargeBehaviour::approachCharger()
{

	float desiredAngle = 0;
	float turnRate = 0.0;
	float velocity = 0.0;
	/*Since the vicon bridge is giving us quaternions, we'll want to get our own yaw back out of it.*/
	double ownYaw = getYaw(ownPose.transform.rotation)
	/*Now we calculate the yaw we'd want from our current position to be driving toward the recharge station.*/
	desiredAngle = getDesiredAngle(chargerX, chargerY, ownPose.position.x, ownPose.position.x;



		/*If we're moving and our current yaw is within 0.25 of what we want, keep driving*/
		if ((ownYaw > (desiredAngle - 0.25)) && (ownYaw < (desiredAngle + 0.25)) && (driving == true))
			
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*However, if we're near the "seam" where ~3.1415 becomes ~-3.1415, we need to be fiddly if our desired angle is on the other side*/
		else if ((((ownYaw > 2.9) && (desiredAngle < -2.9)) || ((ownYaw < -2.9) && (desiredAngle > 2.9))) && (driving == true))	
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
		}
		/*Now, if we're currently turning and have successfully turned to within 0.15 of what we want, start driving again.*/
		else if ((ownYaw > (desiredAngle - 0.15)) && (currentZRotation < (ownYaw + 0.15)) && (driving == false))
			
		{
			move_cmd.linear.x = 0.2;
			move_cmd.angular.z = 0.0;
			driving = true;
		}
		/*And again, if we're within ~0.15 but it's at one of the borders, allow it too.*/
		else if ((((ownYaw > 3) && (desiredAngle < -3)) || ((currentZRotation < -3) && (ownYaw > 3))) && (driving == false))
			
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
void RechargeBehaviour::whileActive(const geometry_msgs::TransformStamped::ConstPtr& pose)
{
	if (chargeLevel < highThreshold && chargeState == 0) 
	{
		/*Change from high charge to mid charge*/
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle STATE CHANGE, 0 to 1, charge level: %f.", chargeLevel);
		chargeState = 1;
	}
	else if (chargeLevel < midThreshold && chargeState == 1) 
	{
		/*Change from mid charge to low charge*/
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle STATE CHANGE, 1 to 2, charge level: %f.", chargeLevel);
		chargeState = 2;
	}
	else if (chargeLevel < lowThreshold && chargeState == 2) /*Below low charge threshold, go recharge*/
	{
		if (abs(ownPose.position.x - chargerX) < 100 && abs(goalPose.position.y - chargerY) < 100)
		{
			/*Activate docking demo by sending signal to /dock*/
			ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle DOCK APPROACHED, charge level: %f.", chargeLevel);
			std_msg::Empty goDock;
			dock_pub.publish(goDock);
			recharging = true;
		}
		else
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle APPROACHING DOCK, charge level: %f.", chargeLevel);
			approachCharger();
		}
	}
	else
	{	
		ROS_INFO("[RECHARGE_BEHAVIOUR] Active Cycle charge level: %f.", chargeLevel);
	}
}

void RechargeBehaviour::whileRecharging(const geometry_msgs::TransformStamped::ConstPtr& pose)
{
	if (chargeLevel > lowThreshold && chargeState == 2) /*Notes highest level of charge*/
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle STATE CHANGE, 2 to 1, charge level: %f.", chargeLevel);
		chargeState = 1;
	}
	else if (chargeLevel > midThreshold && chargeState == 1) /*Crossing from high to middle charge*/
	{
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle STATE CHANGE, 1 to 0, charge level: %f.", chargeLevel);
		chargeState = 0;
	}
	else if (chargeLevel > highThreshold && chargeState == 0) /*Crossing to high charge, time to undock*/
	{
		if (abs(ownPose.position.x - chargerX) > 100 || abs(goalPose.position.y - chargerY) > 100)
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle UNDOCKING COMPLETE, charge level: %f.", chargeLevel);
			recharging = false;
		}
		else
		{
			ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle UNDOCKING, charge level: %f.", chargeLevel);
			std_msg::Empty goDock;
			undock_pub.publish(goDock);
			move_cmd.linear.x = -0.1;
			move_cmd.angular.z = 0.0;
			cmd_vel_pub.publish(move_cmd);
		}
	}
	else
	{	
		ROS_INFO("[RECHARGE_BEHAVIOUR] Recharge Cycle charge level: %f.", chargeLevel);
	}
}

void RechargeBehaviour::spinOnce() {
	if (poseReceived && chargeReceived) {
		if (recharging == false)
		{
			whileActive();
		}
		else
		{
			whileRecharging();
		}
	}
  	ros::spinOnce();
}

void RechargeBehaviour::spin() {
  ros::Rate rate(loopHz);
  while (ros::ok()) {
    spinOnce();
    if (!rate.sleep()) {
      ROS_WARN("[RECHARGE_BEHAVIOUR] Loop running slowly.");
    }
  }
}
