#ifndef RECHARGE_BEHAVIOUR_H
#define RECHARGE_BEHAVIOUR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

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
	double highThreshold;
	double midThreshold;
	double lowThreshold;

	/*state variables*/
	bool  recharging;
	int chargeState;

	/*parameters that name the robot, give it the vicon topic to listen to for its localization and specify where its charger is in the world*/
	double chargerX;
	double chargerY;
	string robotName;
	string viconTopic;

	/*parameters that control when the robot goes to recharge if using time*/
	ros::Time cycleStartTime;
	bool chargeTime;
	double highTime;
	double midTime;
	double lowTime;
	double highChargeTime;
	double midChargeTime;
	double lowChargeTime;

    //void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void ownPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose);
	void chargeLevelCallback(const float charge);

  protected:
    ros::NodeHandle nh;
    ros::Subscriber ownPoseSub;
	ros::Subscriber chargeLevelSub;
    //ros::Publisher posePub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher dock_pub;
	ros::Publisher undock_pub;

    
    // Pose of the target to follow
    //geometry_msgs::PoseStamped targetPose;
    geometry_msgs::TransformStamped ownPose;
    // Pose to publish
    //geometry_msgs::Pose goalPose;
	/*Movement orders for Create*/
	geometry_msgs::Twist move_cmd;


  public:
    RechargeBehaviour(ros::NodeHandle& nh);
    ~RechargeBehaviour();

    virtual void spin();
    virtual void spinOnce();

}; // class RechargeBehaviour

#endif // RECHARGE_BEHAVIOUR_H
