#ifndef RECHARGE_BEHAVIOUR_H
#define RECHARGE_BEHAVIOUR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <vector>

#define PI 3.14159
#define TWO_PI 6.283185

class RechargeBehaviour {
private:
	/*Controls the loop rate*/
 	double loopHz;
	/*The robot's yaw, extracted from the localization subscription*/
 	double yaw;
	/*These two check that we're receiving poses and charge readings from the two subscriptions, the loop doesn't work without them*/
 	bool poseReceived;
	bool chargeReceived;
	/*Used while homing in on recharge location, to differentiate between when you stop to turn toward the target and start driving forward*/
	bool driving;

	/*parameters that control when the robot goes to recharge if using battery level information*/
	double chargeLevel;
	std::vector<double> chargeHistory;
	double chargeLatest;
	double highThreshold;
	double midThreshold;
	double lowThreshold;

	/*state variables*/
	bool recharging;
	int chargeState;
	int buoyPresence;

	/*parameters that name the robot, and specify where its charger is in the world*/
	double chargerX;
	double chargerY1;
	double chargerY2;
	bool chargerPatrolReset;
	std::string robotName;

	/*parameters that control when the robot goes to recharge if using time*/
	ros::Time cycleStartTime;
	ros::Time backupTime;
	ros::Time debugTime;
	bool chargeTime;
	bool backupTimeCheck;
	double highTime;
	double midTime;
	double lowTime;
	double highChargeTime;
	double midChargeTime;
	double lowChargeTime;

  	void ownPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void chargeLevelCallback(const std_msgs::Float32 charge);
	void buoyCallback(const std_msgs::UInt16 irReading);
	float getDesiredAngle(float targetX, float targetY, float currentXCoordinateIn, float currentYCoordinateIn);
	void approachCharger();
	void whileActive();
	void whileRecharging();
	

protected:
  ros::NodeHandle nh;
  ros::NodeHandle privNh;
 	ros::Subscriber ownPoseSub;
	ros::Subscriber chargeLevelSub;
	ros::Subscriber buoySub;

	ros::Publisher cmd_vel_pub;
	ros::Publisher dock_pub;
	ros::Publisher undock_pub;

 	geometry_msgs::TransformStamped ownPose;

	/*Movement orders for Create*/
	geometry_msgs::Twist move_cmd;


public:
  RechargeBehaviour(ros::NodeHandle& nh);
  ~RechargeBehaviour();

  virtual void spin();
  virtual void spinOnce();

}; // class RechargeBehaviour

#endif // RECHARGE_BEHAVIOUR_H
