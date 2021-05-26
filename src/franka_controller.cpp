#include "franka_controller/franka_controller.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "franka_controller");
    ros::NodeHandle n;

    ros::Subscriber jointState = n.subscribe("/mujoco_ros_interface/joint_states", 5, &jointStateCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_sim_command_sub_ = n.subscribe("/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_sim_time_sub_ = n.subscribe("/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_sim_command_pub_ = n.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 5);
    mujoco_joint_set_pub_ = n.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 5);

    mujoco_joint_set_msg_.torque.resize(7);

    ros::Rate loop_rate(100);
    
    while (ros::ok())
    {
      jointTorqueSetCommand();
      mujoco_joint_set_pub_.publish(mujoco_joint_set_msg_);
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    return 0;
}

void jointTorqueSetCommand()
{
    mujoco_joint_set_msg_.MODE = 1;
    mujoco_joint_set_msg_.torque[0] = 200;
    mujoco_joint_set_msg_.torque[1] = 200;
    mujoco_joint_set_msg_.torque[2] = 200;
    mujoco_joint_set_msg_.torque[3] = 200;
    mujoco_joint_set_msg_.torque[4] = 200;
    mujoco_joint_set_msg_.torque[5] = 200;
    mujoco_joint_set_msg_.torque[6] = 200;
    mujoco_joint_set_msg_.header.stamp = ros::Time::now();
    mujoco_joint_set_msg_.time = sim_time;
}

void simCommandCallback(const std_msgs::StringConstPtr &msg)
{

    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        //parameterInitialize();
        mujoco_sim_last_time = 0.0;

        mujoco_ready = true;

        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        mujoco_init_receive = true;
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_.publish(rst_msg_);
        mujoco_sim_time = 0.0;
        mujoco_reset = true;
    }

    if (buf == "terminate")
    {
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg)
{
    mujoco_sim_time = msg->data;
    sim_time = mujoco_sim_time;
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
//    std::cout<< "joint 1 : " << msg->position[0] << std::endl;
//    std::cout<< "joint 2 : " << msg->position[1] << std::endl;
}