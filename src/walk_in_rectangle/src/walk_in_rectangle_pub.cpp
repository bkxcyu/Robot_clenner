#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/Path.h"
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include "walk_in_rectangle/Rectangle.h"


class Walk_in_rectangle
{
public:
	Walk_in_rectangle();
	double rcd;//robot_cleanner_diameter


private:
	ros::NodeHandle n;
	ros::Publisher cmd_pub;
	ros::Publisher path_pub;
	ros::Subscriber rectangle_sub;
	void rectangleCB(const walk_in_rectangle::Rectangle& rectangle);
};

Walk_in_rectangle::Walk_in_rectangle()
{
	ros::NodeHandle pn("~");
	pn.param("rcd", rcd, 0.3);

	rectangle_sub=n.subscribe("/rectangle", 1, &Walk_in_rectangle::rectangleCB, this);
	cmd_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	path_pub=n.advertise<nav_msgs::Path>("/path", 10);
}
void Walk_in_rectangle::rectangleCB(const walk_in_rectangle::Rectangle& rectangle)
{
	double x_dist=fabs(rectangle.U_L.x-rectangle.U_R.x);
	double y_dist=fabs(rectangle.U_L.y-rectangle.D_L.y);
	//ROS_INFO("x_dist:%.2f,y_dist:%.2f",x_dist,y_dist);
	int col=floor(x_dist/rcd);//列
	int row=floor(y_dist/rcd);//行
	//ROS_INFO("col:%d,row:%d",col,row);
	nav_msgs::Path path;
	path.poses.resize((col+1)*(row+1));
	path.header.frame_id="/map";
	path.header.stamp=ros::Time::now();
	for(int r=1;r<=row+1;r++)//行
	{
		for(int c=1;c<=col+1;c++)//列
		{
			geometry_msgs::PoseStamped path_point;
			path_point.header.stamp=ros::Time::now();
			path_point.header.frame_id="map";

			if ( r % 2 == 0)
			{//为偶数行 2 4 6 倒序压栈
				path_point.pose.position.x=rectangle.U_R.x+(rcd/2)-rcd*c;
				path_point.pose.position.y=rectangle.U_L.y+(rcd/2)-rcd*r;
				if(c==col+1 || r==row+1) //边界处理
				{
					if(c==col+1)
					{
						path_point.pose.position.x=rectangle.U_L.x+(rcd/2);
					}
					if(r==row+1)
					{
						path_point.pose.position.y=rectangle.D_L.y+(rcd/2);
					}
				}
				//ROS_INFO("col:%d,row:%d,x:%.2f,y:%.2f",c,r,path_point.pose.position.x,path_point.pose.position.y);
			}
			else
			{//为奇数行 1 3 5 正序压栈
				path_point.pose.position.x=rectangle.U_L.x-(rcd/2)+rcd*c;
				path_point.pose.position.y=rectangle.U_L.y+(rcd/2)-rcd*r;
				if(c==col+1 || r==row+1) //边界处理
				{
					if(c==col+1)
					{
						path_point.pose.position.x=rectangle.U_R.x-(rcd/2);
					}
					if(r==row+1)
					{
						path_point.pose.position.y=rectangle.D_L.y+(rcd/2);
					}
				}
				//ROS_INFO("col:%d,row:%d,x:%.2f,y:%.2f",c,r,path_point.pose.position.x,path_point.pose.position.y);
			}
			path.poses.push_back(path_point);
		}
	}
	path_pub.publish(path);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Walk_in_rectangle");
	Walk_in_rectangle walk_a_walk;
	ros::spin();
    return 0;
}