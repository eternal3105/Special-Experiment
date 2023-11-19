#include "ros/ros.h"                 // ROS 主要功能
#include "std_msgs/String.h"         // 用于接收字符串消息
#include <sys/types.h>               // 包含 pid_t 定义
#include <signal.h>                  // SIGTERM 信号的定义
#include <sys/wait.h>                // waitpid() 函数的定义
#include <unistd.h>                  // fork() 和 execlp() 函数的定义
#include <cstdlib>                   // exit() 函数的定义
#include <unistd.h>

pid_t nav_process_pid = -1;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received command: %s", msg->data.c_str());

    if (msg->data == "start_nav" && nav_process_pid == -1)
    {
        // 收到開始還沒開
        ROS_INFO("Starting navigation process.");
        pid_t pid = fork();
        if (pid == 0) {
            execlp("roslaunch", "roslaunch", "rosky_navigation", "navigation.launch", "map:=kdMap.yaml", "ominibotcar:=true", "local_planner:=teb", (char *)NULL);
            ROS_ERROR("Failed to start navigation.");
            exit(EXIT_FAILURE);
        } else if (pid > 0) {
            nav_process_pid = pid;
        } else {
            ROS_ERROR("Fork failed to create new process.");
        }
    }
    else if (msg->data == "stop_nav" && nav_process_pid != -1)
    {
        // 收到關閉且已經開啟
        ROS_INFO("Stopping navigation process.");
        kill(nav_process_pid, SIGTERM);
        waitpid(nav_process_pid, NULL, 0);
        nav_process_pid = -1;
    }
    else
    {
        ROS_INFO("Command ignored: navigation.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_switch");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/nav_signal", 1000, commandCallback);
    ros::spin();

    return 0;
}
