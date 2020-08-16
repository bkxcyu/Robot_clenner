#include "ros/ros.h"
#include <ros/types.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class SPAWM_RECTANGLE
{
public:
    SPAWM_RECTANGLE();
	nav_msgs::OccupancyGrid map_;
private:
    ros::NodeHandle n;
	ros::Publisher rect_pub,marker_pub;
    ros::Timer timer;
    void testpub_loopCB(const ros::TimerEvent&);
	ros::Subscriber map_sub;
	void mapCB(const nav_msgs::OccupancyGrid& map);
	int8_t get_Occupancy(const geometry_msgs::Point& position);
	void init_marker();
	visualization_msgs::Marker points, line_strip, goal_circle;
	void draw_marker(const geometry_msgs::Point& position);

};

SPAWM_RECTANGLE::SPAWM_RECTANGLE()
{
	map_sub=n.subscribe("/map", 1, &SPAWM_RECTANGLE::mapCB, this);//nav_msgs/OccupancyGrid
	marker_pub = n.advertise<visualization_msgs::Marker>("/Marker", 10);
	timer = n.createTimer(ros::Duration((1.0)/0.5), &SPAWM_RECTANGLE::testpub_loopCB, this);
	init_marker();
    
}
void SPAWM_RECTANGLE::mapCB(const nav_msgs::OccupancyGrid& map)
{	
	map_=map;
	//ROS_INFO("map_.data.siz:%d",map_.data.size());
}
void SPAWM_RECTANGLE::init_marker()
{
	points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "map";
	points.ns = line_strip.ns = goal_circle.ns = "Markers";
	points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
	points.id = 0;
	line_strip.id = 1;
	goal_circle.id = 2;

	points.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	goal_circle.type = visualization_msgs::Marker::CYLINDER;
	// POINTS markers use x and y scale for width/height respectively
	points.scale.x = 0.2;
	points.scale.y = 0.2;

	//LINE_STRIP markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.1;

	goal_circle.scale.x = 0.3;
	goal_circle.scale.y = 0.3;
	goal_circle.scale.z = 0.1;

	// Points are green
	points.color.g = 1.0f;
	points.color.a = 1.0;

	// Line strip is blue
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;

	//goal_circle is yellow
	goal_circle.color.r = 1.0;
	goal_circle.color.g = 1.0;
	goal_circle.color.b = 0.0;
	goal_circle.color.a = 0.5;
}
int8_t SPAWM_RECTANGLE::get_Occupancy(const geometry_msgs::Point& position)
{
	int8_t Occupancy;
	int index;
	int row=(fabs(position.y)/map_.info.resolution);//像素行数
	//ROS_INFO("row:%d",row);
	int col=map_.info.width;//像素列数
	//ROS_INFO("col:%d",col);
	int remain=(fabs(position.x)/map_.info.resolution);
	//ROS_INFO("remain:%d",remain);
	// （y坐标/分辨率）×宽度+（x坐标/分辨率） 	//data[] 为行优先
	index=(int)(row*col+remain);
	ROS_INFO("index:%d",index);

	if(index<=(map_.info.width*map_.info.height))
		Occupancy=map_.data[index];
	else
		ROS_ERROR("index WARM");
		

	if(Occupancy>0)
	{
		ROS_INFO("draw_marker_at:%.1f,%.1f",position.x,position.y);
		draw_marker(position);
	}		
	return Occupancy;
}
void SPAWM_RECTANGLE::draw_marker(const geometry_msgs::Point& position)
{

	points.points.push_back(position);
	marker_pub.publish(points);
}
void SPAWM_RECTANGLE::testpub_loopCB(const ros::TimerEvent&)
{
    geometry_msgs::Point test_point;//25*6
	for(int y=0;y<=60;y++)
	{
		for(int x=0;x<=250;x++)
		{
			test_point.x=1.0*x/10;
			test_point.y=1.0*y/10;
			get_Occupancy(test_point);
		}
	}
	points.points.clear();
	// test_point.x=0;
	// test_point.y=0;
	// get_Occupancy(test_point);
	// test_point.x=1;
	// test_point.y=-1;
	// get_Occupancy(test_point);



}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "spawm_rectangle");
	SPAWM_RECTANGLE spawm;
	ros::spin();
    return 0;
}