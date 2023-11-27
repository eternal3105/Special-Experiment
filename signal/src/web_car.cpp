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
pid_t start_process_pid = -1;
pid_t run_process_pid = -1;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received command: %s", msg->data.c_str());

    if (msg->data == "start" && start_process_pid == -1)
    {
        ROS_INFO("Starting start process.");
        pid_t pid = fork();
        if (pid == 0) {
            execlp("roslaunch", "roslaunch", "showmap", "usb_cam.launch", (char *)NULL);
            ROS_ERROR("Failed to start roslaunch.");
            exit(EXIT_FAILURE);
        } else if (pid > 0) {
            start_process_pid = pid;
        } else {
            ROS_ERROR("Fork failed to create new process.");
        }
    }
    else if (msg->data == "run" && run_process_pid == -1)
    { 
        // runall 不在執行 
        // 如果start进程正在运行，则先关闭它
        if (start_process_pid != -1) {
            sleep(3);
            ROS_INFO("Stopping start process before running runall.");
            kill(start_process_pid, SIGTERM);
            waitpid(start_process_pid, NULL, 0);
            start_process_pid = -1;
        } // if

        // 現在執行runall
        ROS_INFO("Starting runall process.");
        pid_t pid = fork();
        if (pid == 0) {
            execlp("roslaunch", "roslaunch", "/home/rosky01/runall.launch", (char *)NULL);
            ROS_ERROR("Failed to start roslaunch.");
            exit(EXIT_FAILURE);
        } else if (pid > 0) {
            run_process_pid = pid;
        } else {
            ROS_ERROR("Fork failed to create new process.");
        }
    }
    else if ( msg->data == "run" && run_process_pid != -1 )
    {   // runall 正在執行 回到首頁要關掉start

        // 如果start进程正在运行，關閉
        ROS_INFO("Closeing the camera.");
        if (start_process_pid != -1) {
            sleep(3);
            ROS_INFO("Stopping start process before running runall.");
            kill(start_process_pid, SIGTERM);
            waitpid(start_process_pid, NULL, 0);
            start_process_pid = -1;
        } // if
        // run all 不需要執行


    } // else if
    else if (msg->data == "stop" && start_process_pid != -1)
    {
        sleep(3);
        ROS_INFO("Closeing the camera.");
        kill(start_process_pid, SIGTERM);
        waitpid(start_process_pid, NULL, 0);
        start_process_pid = -1;
    }
    else if (msg->data == "navigation" && run_process_pid != -1)
    {
        ROS_INFO("Saving the map before stopping runall.");
        int ret = system("timeout 5s rosrun map_server map_saver -f /home/rosky01/ROSKY/catkin_ws/src/rosky_slam/map/kdMap");
        if (ret != 0) {
            // map_saver 执行失败，可以在这里处理错误
            ROS_WARN("Failed to save the map.");
        }

        sleep(3) ;
        ROS_INFO("Stopping runall and open navigation.");
        kill(run_process_pid, SIGTERM);
        waitpid(run_process_pid, NULL, 0);
        run_process_pid = -1;
    }
    else
    {
        ROS_INFO("Command ignored: either unknown command or process already running.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_talker");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/usb_video", 1000, commandCallback);
    ros::spin();

    return 0;
}


