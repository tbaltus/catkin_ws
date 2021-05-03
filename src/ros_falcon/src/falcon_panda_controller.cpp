#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_falcon/falconForces.h"
#include "sensor_msgs/Joy.h"
#include "math.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

  /*float xpos = 0.0;
  float ypos = 0.0;
  float zpos = 0.0;*/

  double x_panda_force_felt = 0.0;
  double y_panda_force_felt = 0.0;
  double z_panda_force_felt = 0.0; 

/*void chatterCallback(const sensor_msgs::Joy& msg)
{
  //ROS_INFO_STREAM(std::fixed << "position = " << msg.axes[0] << "," << msg.axes[1]<< "," << msg.axes[2]);
  xpos = msg.axes[0];
  ypos = msg.axes[1];
  zpos = msg.axes[2];
}*/

void pandaForceCallback(const geometry_msgs::WrenchStamped& msg)
{
  x_panda_force_felt = msg.wrench.force.x;
  y_panda_force_felt = msg.wrench.force.y;
  z_panda_force_felt = msg.wrench.force.z;

  //ROS_INFO_STREAM(std::fixed << "force = " << msg.wrench.force.x);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "falcon_panda_controller");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/franka_state_controller/F_ext", 1, pandaForceCallback);
  
  ros::Publisher chatter_pub = n.advertise<ros_falcon::falconForces>("/falconForce", 1);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    
    ros_falcon::falconForces msg;
    msg.X = x_panda_force_felt/10;
    msg.Y = y_panda_force_felt/10;
    msg.Z = z_panda_force_felt/10;
    //ROS_INFO_STREAM(std::fixed << "data = " << msg.data[0]);
    /*msg.pose.position.x = (0.5740-0.3740)/(0.06-(-0.06))*xpos+ 0.3740 - (0.5740-0.3740)/(0.06-(-0.06))*(-0.06);
    msg.pose.position.y = (0.1303-(-0.0697))/(0.06-(-0.06))*ypos + (-0.0697)-(0.1303-(-0.0697))/(0.06-(-0.06))*(-0.06);
    msg.pose.position.z = (0.6689-0.4689)/(0.17-0.07)*zpos + 0.4689 - (0.6689-0.4689)/(0.17-0.07)*0.07;// 'panda_link0';*/
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

  }

  //ros::spin();

  return 0;
}