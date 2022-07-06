#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <action_pkg/int_sumAction.h>

typedef actionlib::SimpleActionClient<action_pkg::int_sumAction> Client;

void done_cb(const actionlib::SimpleClientGoalState &state, const action_pkg::int_sumResultConstPtr &result)
{

}


void active_cb()
{
    ROS_INFO("服务激活");
}


void feed_back_cb(const action_pkg::int_sumFeedbackConstPtr &feedback)
{
    ROS_INFO("当前进度%.2f",feedback->progress);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"action_client");

    ros::NodeHandle nh;

    Client client(nh,"sum_int",true);

    client.waitForServer();

    action_pkg::int_sumGoal goal;

    goal.max = 100;

    client.sendGoal(goal,&done_cb,&active_cb,&feed_back_cb);

    ros::spin();

    return 0;
}




