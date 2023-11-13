#include "ros/ros.h"                 // ROS 主要功能
#include "std_msgs/String.h"         // 用於接收字符串消息
#include <sys/types.h>               // 包含 pid_t 定義
#include <signal.h>                  // SIGTERM 信號的定義
#include <sys/wait.h>                // waitpid() 函數的定義
#include <unistd.h>                  // fork() 和 execlp() 函數的定義
#include <cstdlib>                   // exit() 函數的定義
#include <unistd.h>

/*
                網頁發送topic給ROSKY開啟攝影機                          車發送topic給server開啟校正+環景
網頁點擊監控或環景 ---------------------------> 車子開啟usb_cam.launch ----------------------------------> server開啟open.launch

            網頁發送topic給車子關閉usb_cam  
網頁點擊地圖 -----------------------------> server關閉open.launch ----> 車子關閉usb_cam.launch

想要的
1. 監控
2. 環景
    a. 切換
3. 地圖

4. 如何開導航

*/

pid_t current_process_pid = -1;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received command: %s", msg->data.c_str());

    // 如果收到的命令是 "stop"
    if (msg->data == "stop" && current_process_pid != -1)
    {
        sleep( 3 ) ;
        ROS_INFO("Stopping current process.");
        // 發送 SIGTERM 信號給當前進程
        kill(current_process_pid, SIGTERM);
        // 等待子進程終止
        waitpid(current_process_pid, NULL, 0);
        current_process_pid = -1;
    }
    else if (msg->data == "start" && current_process_pid == -1)
    {
        ROS_INFO("Starting new process.");
        // 使用 fork() 創建一個新進程
        pid_t pid = fork();
        if (pid == 0) // 子進程
        {

            // 在子進程中執行指定的 roslaunch 命令
            execlp("roslaunch", "roslaunch", "showmap", "usb_cam.launch", (char *)NULL);
            // 如果execlp返回，則表明有錯誤發生
            ROS_ERROR("Failed to start roslaunch.");
            exit(EXIT_FAILURE); // 使用錯誤代碼退出子進程
        }
        else if (pid > 0) // 主進程
        {
            current_process_pid = pid;
        }
        else
        {
            // fork 失敗
            ROS_ERROR("Fork failed to create new process.");
        }
    }
    else if (msg->data == "start" && current_process_pid != -1)
    {
        // 如果已有進程在運行，則不允許重複啟動進程
        ROS_INFO("A process is already running. Cannot start another one.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("command_listener", 1000, commandCallback);
    ros::spin();

    return 0;
}

