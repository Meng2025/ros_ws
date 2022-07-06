#include <ros/ros.h>
#include <serial_pkg/serial_commu.h>

#include <sensor_msgs/JointState.h>


Arm arm;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg_p)
{
    float set_joint[5];

    set_joint[0] = msg_p->position[0];
    set_joint[1] = msg_p->position[1];
    set_joint[2] = msg_p->position[2];
    set_joint[3] = msg_p->position[3];
    set_joint[4] = msg_p->position[5];

    arm.set(set_joint);

    // ROS_INFO("%f %f %f %f %f",msg_p->position[0],msg_p->position[1],msg_p->position[2],msg_p->position[3],msg_p->position[4]);

}


int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc,argv,"arm_driver");

    ros::NodeHandle nh;

    ros::Rate r(1);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",10,joint_state_callback);

    ros::spin();


    // while(ros::ok())
    // {
    //     int set_id = 1;
    //     float set_degree = 0.0;
    //     uint16_t set_time = 1000;
        
    //     std::cout<<"id degree time :";
    //     std::cin>>set_id>>set_degree>>set_time;

    //     arm.set1(set_id,set_degree,set_time);

    //     ROS_INFO("%d %f %d",set_id,set_degree,set_time);

    //     r.sleep();
    // }


    // while(ros::ok())
    // {
    //     int set_id = 1;
    //     uint16_t set_degree = 2048;
    //     uint16_t set_time = 1000;
    //     std::cout<<"id degree time :";
    //     std::cin>>set_id>>set_degree>>set_time;

    //     ROS_INFO("%d %d %d",set_id,set_degree,set_time);

    //     arm.set1(set_id, set_degree, set_time);

    //     r.sleep();
    // }

    return 0;
}





