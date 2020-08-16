#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "walk_in_rectangle/Rectangle.h"

class RectanglePub
{
public:
    RectanglePub();
private:
    ros::NodeHandle n;
	ros::Publisher rect_pub;
    walk_in_rectangle::Rectangle rect;
    ros::Timer timer;
    void testpub_loopCB(const ros::TimerEvent&);

};

RectanglePub::RectanglePub()
{
    rect_pub=n.advertise<walk_in_rectangle::Rectangle>("/rectangle", 10);
    timer = n.createTimer(ros::Duration((1.0)/10), &RectanglePub::testpub_loopCB, this);
    rect.U_L.x=0;       rect.U_R.x=25;
    rect.U_L.y=6;       rect.U_R.y=6;
    
    rect.D_L.x=0;       rect.D_R.x=25;
    rect.D_L.y=0;       rect.D_R.y=0;
    
    
}
void RectanglePub::testpub_loopCB(const ros::TimerEvent&)
{
    rect_pub.publish(rect);
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rect_pub");
	RectanglePub rect_publisher;
	ros::spin();
    return 0;
}