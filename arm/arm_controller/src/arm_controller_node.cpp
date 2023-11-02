
// arm_controller_node.cpp file for step 4

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>


void chatterCallback(const sensor_msgs::JointState::ConstPtr& joint_states)

{
   for (int i=0; i< joint_states->name.size(); i++){
   ROS_INFO("Position of joint %s:\n[%f]\n",  joint_states->name[i].c_str(), joint_states->position[i]);
   }
}


int main(int argc, char* argv[])

{

ros::init(argc, argv, "arm_controller_node");
ros::NodeHandle nodeHandle;

ros::Subscriber subscriber = nodeHandle.subscribe("/arm/joint_states",10,chatterCallback);    // Subscriber for step 4b

ros::Publisher j0_command_pub, j1_command_pub, j2_command_pub, j3_command_pub;                // Publishers for step 4c

j0_command_pub = nodeHandle.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J0_controller/command", 10);
j1_command_pub = nodeHandle.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J1_controller/command", 10);
j2_command_pub = nodeHandle.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J2_controller/command", 10);
j3_command_pub = nodeHandle.advertise<std_msgs::Float64>("/arm/PositionJointInterface_J3_controller/command", 10);

ros::Rate loopRate(10);

while (ros::ok()) {

    std_msgs::Float64 j0_command;
    j0_command.data = 0.5;
    j0_command_pub.publish(j0_command);

    std_msgs::Float64 j1_command;
    j1_command.data = -1.7;
    j1_command_pub.publish(j1_command);

    std_msgs::Float64 j2_command;
    j2_command.data = 1.2;
    j2_command_pub.publish(j2_command);

    std_msgs::Float64 j3_command;
    j3_command.data = 0.7;
    j3_command_pub.publish(j3_command);

    ros::spinOnce();
    loopRate.sleep();

}

return 0;

}
