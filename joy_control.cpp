// mode2，原本是用左蘑菇頭控制前後，改為ZL，ZR代表前後
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>

#include <string>
#include <vector>

using namespace std;

class Car_control {
   
   public:

    geometry_msgs::Twist control(ros::Publisher publisher, float angular, int vol_front, int vol_back) {
        geometry_msgs::Twist twist;



        if ( vol_front != 0 || vol_back != 0 ) {
            twist.linear.x = ( vol_front - vol_back ) * 0.3 ;
        } // if()

        if ( angular != 0 ) {
            if ( angular > 0 ) twist.angular.z = 0.3 ;
            else twist.angular.z = -0.3 ;
        } // if()
        
        
        return twist;
    }
};


class Joy_rosky {
   private:
    int ZL, ZR, RMLR_index, LMUD_index;
    double velocity, rotational;

    ros::Publisher publisher;

    ros::Subscriber subscriber;

   public:
    Joy_rosky() {
        ros::NodeHandle nodePtr;
        nodePtr.param<int>("ZL", ZL, 8);                 // ZL, for button
        nodePtr.param<int>("ZR", ZR, 9);                 // ZR, for button
        nodePtr.param<int>("RMLR_index", RMLR_index, 2); // right mushroom ( left right ), for axes
        nodePtr.param<int>("LMUD_index", LMUD_index, 1); // left mushroom ( up down ), for axes
        

        // create a publisher that will advertise on the command_velocity topic of the turtle

        publisher = nodePtr.advertise<geometry_msgs::Twist>("/rosky01/cmd_vel", 1);

        // subscribe to the joystick topic for the input to drive the turtle
        subscriber = nodePtr.subscribe<sensor_msgs::Joy>("joy", 1, &Joy_rosky::joyCallback, this);

        // subscriber_collision = nodePtr.subscribe("collision", 1, &Joy_rosky::collisionCallback, this);
    }

   private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        Car_control controller;
        geometry_msgs::Twist twist = move(controller.control(publisher, joy->axes[RMLR_index], joy->buttons[ZR], joy->buttons[ZL]));
        publisher.publish(twist);
    }

    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple");
    Joy_rosky joy_rosky;
    ros::spin();
}