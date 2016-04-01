#include "recharge_behaviour/recharge_behaviour.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "recharge_behaviour");
  ros::NodeHandle nh;

  RechargeBehaviour rechargeBehaviour(nh);

  try {
    rechargeBehaviour.spin();
  }
  catch (std::runtime_error& ex) {
    ROS_FATAL_STREAM("[RECHARGE_BEHAVIOUR] Runtime error: " << ex.what());
    return 1;
  }

  return 0;
}
