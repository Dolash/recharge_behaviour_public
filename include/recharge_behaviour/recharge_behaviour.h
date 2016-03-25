#ifndef RECHARGE_BEHAVIOUR_H
#define RECHARGE_BEHAVIOUR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.14159
#define TWO_PI 6.283185

class RechargeBehaviour {
  private:
    double loopHz;
    double distX;
    double distY;
    double distZ;
    double yaw;
    bool poseReceived;
	bool chargeReceived;
	bool driving;

	double chargeLevel;
	double highThreshold;
	double midThreshold;
	double lowThreshold;
	bool  recharging;
	int chargeState;
	double chargerX;
	double chargerY;

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
