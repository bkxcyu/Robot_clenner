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
#include <tf/transform_broadcaster.h>


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
	tf::TransformBroadcaster br;
	tf::Transform transform;

	void rectangleCB(const walk_in_rectangle::Rectangle& rectangle);
};

Walk_in_rectangle::Walk_in_rectangle()
{
	ros::NodeHandle pn("~");
	pn.param("rcd", rcd, 0.3);

    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/currant_path_plan_origion"));

	rectangle_sub=n.subscribe("/rect_spawm", 1, &Walk_in_rectangle::rectangleCB, this);
	cmd_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	path_pub=n.advertise<nav_msgs::Path>("/path", 10);
}
void Walk_in_rectangle::rectangleCB(const walk_in_rectangle::Rectangle& rectangle)
{
    transform.setOrigin( tf::Vector3(rectangle.U_L.x, rectangle.U_L.y, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/currant_path_plan_origion"));
	
	nav_msgs::Path path;
	path.header.frame_id="/currant_path_plan_origion";
	path.header.stamp=ros::Time::now();

	// for(int i=0;i<path.poses.size();i++)
	// {
	// 	path.poses[i].pose.position.x=rectangle.U_L.x-last_path_plan_origion.x;
	// 	path.poses[i].pose.position.y=rectangle.U_L.y-last_path_plan_origion.y;
	// }
	// last_path_plan_origion=rectangle.U_L;

	double x_dist=fabs(rectangle.U_L.x-rectangle.U_R.x);
	double y_dist=fabs(rectangle.U_L.y-rectangle.D_L.y);
	//ROS_INFO("x_dist:%.2f,y_dist:%.2f",x_dist,y_dist);
	int col=floor(x_dist/rcd);//列
	int row=floor(y_dist/rcd);//行
	//ROS_INFO("col:%d,row:%d",col,row);
	for(int r=1;r<=row+1;r++)//行
	{
		for(int c=1;c<=col+1;c++)//列
		{
			geometry_msgs::PoseStamped path_point;
			path_point.header.stamp=ros::Time::now();
			path_point.header.frame_id=path.header.frame_id;

			if ( r % 2 == 0)
			{//为偶数行 2 4 6 倒序压栈
				path_point.pose.position.x=x_dist+(rcd/2)-rcd*c;
				path_point.pose.position.y=(rcd/2)-rcd*r;
				if(c==col+1 || r==row+1) //边界处理
				{
					if(c==col+1)
					{
						path_point.pose.position.x=(rcd/2);
					}
					if(r==row+1)
					{
						path_point.pose.position.y=-y_dist+(rcd/2);
					}
				}
				//ROS_INFO("col:%d,row:%d,x:%.2f,y:%.2f",c,r,path_point.pose.position.x,path_point.pose.position.y);
			}
			else
			{//为奇数行 1 3 5 正序压栈
				path_point.pose.position.x=0-(rcd/2)+rcd*c;
				path_point.pose.position.y=0+(rcd/2)-rcd*r;
				if(c==col+1 || r==row+1) //边界处理
				{
					if(c==col+1)
					{
						path_point.pose.position.x=x_dist-(rcd/2);
					}
					if(r==row+1)
					{
						path_point.pose.position.y=-y_dist+(rcd/2);
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