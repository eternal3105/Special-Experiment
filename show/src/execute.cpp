#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nav_msgs/OccupancyGrid.h>

pid_t current_process_pid = -1;

ros::Publisher pub;

void createOccupancyGrid()
{
    int width = 1984;
    int height = 1984;
    int8_t value = 100;

    nav_msgs::OccupancyGrid grid;
    
    // 设置地图的大小
    grid.info.width = width;
    grid.info.height = height;

    // 设置地图的分辨率（你可能需要设置为适当的值，这里只是示例）
    grid.info.resolution = 0.05;  // 5cm per cell

    // 分配足够的空间并将所有值设置为-1
    grid.data.resize(width * height, value);


    pub.publish(grid);
    ros::spinOnce();  // 处理任何等待的回调函数
    ros::Rate rate(10);  // 10Hz
    rate.sleep();  // 等待足够的时间确保消息已被发送
}

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received command: %s", msg->data.c_str());

    if (msg->data == "stop")
    {
        // system(msg->data.c_str()); save
        // system(msg->data.c_str()); send
        if (current_process_pid != -1) {
            ROS_INFO("Stopping current process group.");
            // Kill the entire process group
            killpg(current_process_pid, SIGTERM);
            waitpid(current_process_pid, NULL, 0);
            current_process_pid = -1;
        }
        createOccupancyGrid();
        return;
    }

    if (current_process_pid == -1)
    {
        pid_t pid = fork();

        if (pid == 0) // Child process
        {
            // Set the child process to be the leader of its own process group
            setpgid(0, 0);

            system(msg->data.c_str());
            exit(0);
        }
        else if (pid > 0) // Parent process
        {
            // Set the child process to be the leader of its own process group
            setpgid(pid, pid);
            
            current_process_pid = pid;
        }
        else
        {
            ROS_ERROR("Fork failed.");
        }
    }
    else
    {
        ROS_INFO("Command ignored as another process is running.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_listener");
    ros::NodeHandle n;

    pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Subscriber sub = n.subscribe("command", 1000, commandCallback);
    ros::spin();

    return 0;
}
