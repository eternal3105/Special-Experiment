#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
using namespace std;

ros::NodeHandle* nh;

string machine_name_1, machine_name_2;

class Basic {
   protected:
    ros::Subscriber subscriber;
};

class Gateway : public Basic {
   private:
    ros::Publisher publisher_1, publisher_2;

    static int active_machine;

   public:
    Gateway() {
        nh->param<string>("machine_name_1", machine_name_1, "rosky");
        nh->param<string>("machine_name_2", machine_name_2, "turtlebot");
        subscriber = nh->subscribe<sensor_msgs::Joy>("/joy", 1, &Gateway::callback, this);
        publisher_1 = nh->advertise<sensor_msgs::Joy>(machine_name_1 + "/joy", 1);
        publisher_2 = nh->advertise<sensor_msgs::Joy>(machine_name_2 + "/joy", 1);
    }

    void callback(const sensor_msgs::Joy::ConstPtr& msg) {
        if (active_machine == 0) {
            publisher_1.publish(msg);
        } else if (active_machine == 1) {
            publisher_2.publish(msg);
        }
    }

    friend class Mux;
};

int Gateway::active_machine = 0;

class Mux : Basic {
   public:
    Mux() {
        subscriber = nh->subscribe<std_msgs::String>("/web/feedback", 1, &Mux::callback, this);
    }

    void callback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == machine_name_1) {
            Gateway::active_machine = 0;
        } else if (msg->data == machine_name_2) {
            Gateway::active_machine = 1;
        } else {
            Gateway::active_machine = -1;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mux");
    nh = new ros::NodeHandle;
    Gateway gateway;
    Mux mux;
    ros::spin();
}