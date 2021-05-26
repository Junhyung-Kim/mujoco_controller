#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "mujoco_ros_msgs/JointSet.h"


ros::Publisher mujoco_sim_command_pub_;
ros::Publisher mujoco_joint_set_pub_;
mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void simCommandCallback(const std_msgs::StringConstPtr &msg);
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void jointTorqueSetCommand();

bool sim_runnung;
bool mujoco_ready = false;
bool mujoco_init_receive = false;
bool mujoco_reset = false;
float mujoco_sim_time;
float mujoco_sim_last_time;
double sim_time;