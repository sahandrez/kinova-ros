#include "ros/ros.h"
#include <signal.h>
//#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_msgs/SetTorqueControlMode.h"

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "emergency_stop", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<kinova_msgs::SetTorqueControlMode>("/j2n6s300_driver/in/set_torque_control_mode");

  kinova_msgs::SetTorqueControlMode srv;

  signal(SIGINT, mySigIntHandler);

  ROS_INFO("***** Emergency stop node started ******");
  ROS_INFO("In case things got out of hands, kill this terminal with Ctrl-C !!");

  while (!g_request_shutdown)
  {
    // this loop works until ctrl-c is pressed
    ros::spinOnce();
    usleep(100000);
  }

  // pre-shutdown task: switch the controller to position control
  srv.request.state = 0;
  client.call(srv);

  ROS_INFO("Switched back to position control. Everybody is safe now!");

  ros::shutdown();
}
