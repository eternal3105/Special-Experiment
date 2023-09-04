#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>

#include <string>
#include <vector>

using namespace std;

enum Position {
    top,
    top_R,
    top_L,
    bottom,
    bottom_R,
    bottom_L,
    top_slow,
    bottom_slow
};

class Trigger {
   protected:
    int previousValue;

   public:
    Trigger() {
        previousValue = 0;
    }

    bool isActive(int currentValue) {
        bool returnValue = false;
        if (previousValue != 0 && currentValue == 0) {
            returnValue = true;
        }

        previousValue = currentValue;
        return returnValue;
    }
};
class Argument {
   public:
    int gear;

    bool ready;
    vector<pair<float, float> > record;
    Trigger trigger_R_bottom, trigger_B_bottom, trigger_A_bottom;

    bool marco_REC;
    bool back;

    Argument() {
        marco_REC = false;
        back = false;
    }
};

class Car_control {
   private:
    static Argument argument;
    static int gearNum;

   public:
    static bool active_avoidance;

    Car_control() {
        gearNum = 5;
    }

    void gear_shift(int lb, int rb) {
        if (lb == 0 && rb == 0) {
            argument.ready = true;
        }

        if (argument.ready) {
            if (lb ^ rb) {
                if (lb == 1) {
                    argument.gear--;
                } else {
                    argument.gear++;
                }

                argument.ready = false;
            }
        }

        if (argument.gear < 0) {
            argument.gear = 0;
        } else if (argument.gear >= gearNum) {
            argument.gear = gearNum - 1;
        }
    }

    geometry_msgs::Twist control(ros::Publisher publisher, float angular, float vol_forward, float vol_backward, int lb, int rb, int RB, int B, int A, double velocity, double rotational) {
        geometry_msgs::Twist twist;
        twist.linear.x = (vol_forward - vol_backward) * velocity * (argument.gear + 1) / gearNum;
        twist.angular.z = angular * rotational * (argument.gear + 1) / gearNum;


        bool R_bottom_state = argument.trigger_R_bottom.isActive(RB);
        bool B_bottom_state = argument.trigger_B_bottom.isActive(B);
        bool A_bottom_state = argument.trigger_A_bottom.isActive(A);

        gear_shift(lb, rb);
        if (R_bottom_state) {
            argument.record.clear();
        }

        if (B_bottom_state) {
            argument.back = true;
        }

        if (A_bottom_state) {
            active_avoidance = !active_avoidance;
        }

        if (argument.back) {
            if (argument.record.size() > 0) {
                int index = argument.record.size() - 1;
                twist.linear.x = argument.record[index].first;
                twist.angular.z = argument.record[index].second;
                argument.record.erase(argument.record.begin() + index);
                return twist;
            } else {
                argument.back = false;
            }
        } else {
            if (abs(twist.linear.x) > 0.01 || abs(twist.angular.z) > 0.1)
                argument.record.push_back(move(pair<float, float>(-twist.linear.x, -twist.angular.z)));
        }

        return twist;
    }
};

int Car_control::gearNum;
bool Car_control::active_avoidance = true;

Argument Car_control::argument;
class Joy_rosky {
   private:
    int RT_index, LT_index, axis_index, LB_index, RB_index, option_index, B, A;
    double velocity, rotational;
    string ifCollision;

   public:
    Joy_rosky() {
        ros::NodeHandle nodePtr;
        nodePtr.param<int>("RT_index", RT_index, 4);
        nodePtr.param<int>("LT_index", LT_index, 3);
        nodePtr.param<int>("axis_index", axis_index, 0);
        nodePtr.param<int>("LB_index", LB_index, 4);
        nodePtr.param<int>("RB_index", RB_index, 5);
        nodePtr.param<int>("option_index", option_index, 7);
        nodePtr.param<int>("B", B, 1);
        nodePtr.param<int>("A", A, 2);
        nodePtr.param<double>("velocity", velocity, 0.11);
        nodePtr.param<double>("rotational", rotational, 0.45);

        // create a publisher that will advertise on the command_velocity topic of the turtle

        publisher = nodePtr.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	    // publisher = nodePtr.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);

        // subscribe to the joystick topic for the input to drive the turtle
        subscriber = nodePtr.subscribe<sensor_msgs::Joy>("joy", 1, &Joy_rosky::joyCallback, this);

        subscriber_collision = nodePtr.subscribe("collision", 1, &Joy_rosky::collisionCallback, this);
    }

   private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        Car_control controller;
        geometry_msgs::Twist twist = move(controller.control(publisher, joy->axes[axis_index], (-joy->axes[RT_index] + 1) / 2, (-joy->axes[LT_index] + 1) / 2, joy->buttons[LB_index], joy->buttons[RB_index], joy->buttons[option_index], joy->buttons[B], joy->buttons[A], velocity, rotational));
        std_msgs::Int8 msg;

        if (Car_control::active_avoidance) {
            if (twist.linear.x > 0) {
                if (collisionState[top] == 0) {
                    twist.linear.x = 0;
                } else if (collisionState[top_slow] == 0 && twist.linear.x > 0.05) {
                    twist.linear.x *= 0.5;
                }

            } else {
                if (collisionState[bottom] == 0) {
                    twist.linear.x = 0;
                } else if (collisionState[bottom_slow] == 0 && twist.linear.x < -0.05) {
                    twist.linear.x *= 0.5;
                }
            }

            if (twist.angular.z < 0) {
                if (collisionState[top_R] == 0 || collisionState[bottom_L] == 0) {
                    twist.angular.z = 0;
                }
            } else {
                if (collisionState[top_L] == 0 || collisionState[bottom_R] == 0) {
                    twist.angular.z = 0;
                }
            }
        }

        publisher.publish(twist);
    }

    int collisionState[8];
    void collisionCallback(const std_msgs::Int16MultiArray& msg) {
        for (int i = 0; i < 8; i++) {
            collisionState[i] = msg.data[i];
        }
    }

    ros::Publisher publisher;

    ros::Subscriber subscriber;
    ros::Subscriber subscriber_collision;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_control");
    Joy_rosky joy_rosky;
    ros::spin();
}

