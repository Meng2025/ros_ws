#include <ros/ros.h>
#include <iostream>
#include <fun_call_pkg/test_info.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "call_info");

    Test test;

    test.output();
    
    return 0;
}




