#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_pkg/int_sumAction.h>


typedef actionlib::SimpleActionServer<action_pkg::int_sumAction> Server;

void call_back(const action_pkg::int_sumGoalConstPtr &goal, Server* server)
{
    int max = goal->max;
    ROS_INFO("目标值： %d",max);

    int result = 0;

    action_pkg::int_sumFeedback feedback;

    ros::Rate rate(10);

    /* 执行过程 */
    for (int i=1;i<max;i++)
    {
        result += i;
        feedback.progress = i/ (double)max;
        server->publishFeedback(feedback);
        rate.sleep();
    }

    /* 最终结果 */
    action_pkg::int_sumResult r;
    r.result = result;
    server->setSucceeded(r);

    ROS_INFO("最终结果：%d",r.result);

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc, argv, "action_server");

    ros::NodeHandle nh;

    Server server(nh,"sum_int",boost::bind(&call_back,_1,&server),false);

    server.start();

    ros::spin();

    return 0;
}




