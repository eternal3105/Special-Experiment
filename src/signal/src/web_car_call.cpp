#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <cstdlib> // for system() function
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

ros::Publisher pub ;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received command: %s", msg->data.c_str());
    
    // 如果收到的命令是 "stop"
    if ( msg->data == "stop" )
    {
        std_msgs::String pub_msg ;
        pub_msg.data = "stop";
        pub.publish(pub_msg);
        return;
    }

    else {
      std_msgs::String pub_msg ;
      pub_msg.data = "start";
      
      pub.publish(pub_msg);
    } // else()

    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intermediate_1");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("command_talker", 1000, commandCallback);
    pub = n.advertise<std_msgs::String>("command_listener", 1);

        
    ros::spin();

    return 0;
}
