#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "linear_pub");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int32>("linear", 10);
    ROS_INFO("Publishing to topic linear...");

    ros::Rate rate(0.1); // 0.1 Hz, 10 seconds
    sleep(10); // wait for 10 seconds
    std_msgs::Int32 msg;

    while (ros::ok())
    {
        char ch;
        cin>>ch;
        switch(ch){
            case 'o' :
            case 'O' :
                msg.data = 1900;
                break;
            case 'p' :
            case 'P' :
                msg.data = 1035;
                break;
        }
        
        ROS_INFO("Published message: %d", msg.data);

        pub.publish(msg);
        ros::spinOnce();
    }

}