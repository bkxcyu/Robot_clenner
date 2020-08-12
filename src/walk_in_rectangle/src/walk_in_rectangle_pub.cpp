#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

class walk_in_rectangle
{
public:
	walk_in_rectangle();

private:
	/* data */
};

walk_in_rectangle::walk_in_rectangle()
{

}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "walk_in_rectangle");
	walk_in_rectangle walk_a_walk();
	ros::spin();
    return 0;
}