#include "ros/ros.h"
#include <ros/types.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class SPAWM_RECTANGLE
{
public:
    SPAWM_RECTANGLE();
	double rcd;//robot_cleanner_diameter
	double start_pose_x,start_pose_y;//the start pose of the robot
	double map_data_origion_x;
	double map_data_origion_y;

private:
    ros::NodeHandle n;
	nav_msgs::OccupancyGrid map_;
	bool map_receive_flag;
	ros::Publisher rect_pub,marker_pub;
    ros::Timer timer;
    void testpub_loopCB(const ros::TimerEvent&);
	void spawm_loopCB(const ros::TimerEvent&);
	ros::Subscriber map_sub;
	void mapCB(const nav_msgs::OccupancyGridConstPtr& map);
	int8_t get_Occupancy(const geometry_msgs::Point& position);
	int8_t get_Occupancy(const geometry_msgs::Pose& pose);
	int8_t get_Occupancy(const float& x,const float& y);
	void init_marker();
	void init_spawm();
	visualization_msgs::Marker points, line_strip, goal_circle;
	void draw_marker(const geometry_msgs::Point& position);
	void draw_marker(const geometry_msgs::Pose& pose);
	void draw_marker(const geometry_msgs::PoseArray& pose_array);
	geometry_msgs::PoseArray spawm();
	geometry_msgs::PoseArray get_poses_between_peaks(geometry_msgs::Pose& peak1,geometry_msgs::Pose& peak2);

	geometry_msgs::Pose up_left_peak;
	geometry_msgs::Pose up_right_peak;
	geometry_msgs::Pose down_left_peak;
	geometry_msgs::Pose down_right_peak;
	geometry_msgs::PoseArray edges;
	bool scan_up_left=true;
	bool scan_down_left=true;
	bool scan_up_right=true;
	bool scan_down_right=true;
	bool scan_up=true;
	bool scan_down=true;
	bool scan_left=true;
	bool scan_right=true;
	double step;

};

SPAWM_RECTANGLE::SPAWM_RECTANGLE()
{
	ros::NodeHandle pn("~");
	pn.param("rcd", rcd, 0.3);//start_pose_x
	pn.param("start_pose_x", start_pose_x, 1.0);
	pn.param("start_pose_y", start_pose_y, 1.0);
	pn.param("map_data_origion_x", map_data_origion_x, 0.0);
	pn.param("map_data_origion_y", map_data_origion_y, 0.0);
	pn.param("step", step, 0.2);
	ROS_INFO("\n rcd:%.1f,start_pose_x:%.1f,start_pose_y:%.1f,step:%.2f \n",rcd,start_pose_x,start_pose_y,step);

	map_receive_flag=false;

	map_sub=n.subscribe("/map", 1, &SPAWM_RECTANGLE::mapCB, this);//nav_msgs/OccupancyGrid
	marker_pub = n.advertise<visualization_msgs::Marker>("/Marker", 10);

	init_spawm();
	timer = n.createTimer(ros::Duration((1.0)/20), &SPAWM_RECTANGLE::spawm_loopCB, this);
	//timer = n.createTimer(ros::Duration((1.0)/0.5), &SPAWM_RECTANGLE::testpub_loopCB, this);

	init_marker();

	// ros::Duration(1.0).sleep();

}
void SPAWM_RECTANGLE::mapCB(const nav_msgs::OccupancyGridConstPtr& map)
{	
	map_=*map;
	//map_.info.resolution=0.05;
	map_receive_flag=true;
	ROS_INFO("map received");
	// ROS_INFO("map_.data.siz:%d,map_.info.resolutio:%.3f\n",map_.data.size(),map_.info.resolution);

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
	int8_t Occupancy=-1;
	if(map_.info.resolution==0)
	{
		ROS_WARN("map_.info.resolution=%.3f,unknow Occupancy",map_.info.resolution);
		return Occupancy;
	}
	int index;
	int row=(fabs(position.y-map_data_origion_y)/map_.info.resolution);//像素行数
	//ROS_INFO("row:%d",row);
	int col=map_.info.width;//像素列数
	//ROS_INFO("col:%d",col);
	int remain=(fabs(position.x-map_data_origion_x)/map_.info.resolution);
	//ROS_INFO("remain:%d",remain);
	// （y坐标/分辨率）×宽度+（x坐标/分辨率） 	//data[] 为行优先
	index=(int)(row*col+remain);
	//ROS_INFO("index:%d,width:%d,height:%d",index,map_.info.width,map_.info.height);

	if(index<=(map_.info.width*map_.info.height))
	{
		Occupancy=map_.data[index];
		//ROS_INFO("index:%d,Occupancy:%d",index,Occupancy);
	}
	else
		ROS_ERROR("index WARN,position:[%.2f,%.2f],map_.info.resolution:%.2f,row:%d,col:%d,remain:%d",position.x,position.y,map_.info.resolution,row,col,remain);
		
	// if(Occupancy>0)
	// {
	// 	ROS_INFO("draw_marker_at:%.1f,%.1f",position.x,position.y);
	// 	draw_marker(position);
	// }		
	return Occupancy;
}
int8_t SPAWM_RECTANGLE::get_Occupancy(const geometry_msgs::Pose& pose)
{
	int8_t Occupancy=-1;
	if(map_.info.resolution==0)
	{
		ROS_WARN("map_.info.resolution=%.3f,unknow Occupancy",map_.info.resolution);
		return Occupancy;
	}
	float x=pose.position.x;
	float y=pose.position.y;
	int index;
	int row=(fabs(y-map_data_origion_y)/map_.info.resolution);//像素行数
	//ROS_INFO("row:%d",row);
	int col=map_.info.width;//像素列数
	//ROS_INFO("col:%d",col);
	int remain=(fabs(x-map_data_origion_x)/map_.info.resolution);
	//ROS_INFO("remain:%d",remain);
	// （y坐标/分辨率）×宽度+（x坐标/分辨率） 	//data[] 为行优先
	index=(int)(row*col+remain);
	//ROS_INFO("index:%d,width:%d,height:%d",index,map_.info.width,map_.info.height);

	if(index<=(map_.info.width*map_.info.height))
	{
		Occupancy=map_.data[index];
		//ROS_INFO("index:%d,Occupancy:%d",index,Occupancy);
	}
	else
		ROS_ERROR("index WARN");
		
	// if(Occupancy>0)
	// {
	// 	ROS_INFO("draw_marker_at:%.1f,%.1f",x,y);
	// 	draw_marker(pose.position);
	// }		
	return Occupancy;
}
int8_t SPAWM_RECTANGLE::get_Occupancy(const float& x,const float& y)
{
	int8_t Occupancy=-1;
	if(map_.info.resolution==0)
	{
		ROS_WARN("map_.info.resolution=%.3f,unknow Occupancy",map_.info.resolution);
		return Occupancy;
	}
	int index;
	int row=(fabs(y-map_data_origion_y)/map_.info.resolution);//像素行数
	//ROS_INFO("row:%d",row);
	int col=map_.info.width;//像素列数
	//ROS_INFO("col:%d",col);
	int remain=(fabs(x-map_data_origion_x)/map_.info.resolution);
	//ROS_INFO("remain:%d",remain);
	// （y坐标/分辨率）×宽度+（x坐标/分辨率） 	//data[] 为行优先
	index=(int)(row*col+remain);
	//ROS_INFO("index:%d,width:%d,height:%d",index,map_.info.width,map_.info.height);

	if(index<=(map_.info.width*map_.info.height))
	{
		Occupancy=map_.data[index];
		//ROS_INFO("index:%d,Occupancy:%d",index,Occupancy);
	}
	else
		ROS_ERROR("index WARN");
		
	// if(Occupancy>0)
	// {
	// 	ROS_INFO("draw_marker_at:%.1f,%.1f",x,y);
	// 	geometry_msgs::Point marker_point;
	// 	marker_point.x=x;
	// 	marker_point.y=y;
	// 	draw_marker(marker_point);
	// }		
	return Occupancy;
}
void SPAWM_RECTANGLE::draw_marker(const geometry_msgs::Point& position)
{
	//points.points.clear();
	points.points.push_back(position);
	marker_pub.publish(points);
}
void SPAWM_RECTANGLE::draw_marker(const geometry_msgs::Pose& pose)
{
	//points.points.clear();
	points.points.push_back(pose.position);
	marker_pub.publish(points);
}
void SPAWM_RECTANGLE::draw_marker(const geometry_msgs::PoseArray& pose_array)
{
	//points.points.clear();
	for(geometry_msgs::Pose each_pose:pose_array.poses)
	{
		points.points.push_back(each_pose.position);
	}
	marker_pub.publish(points);
}
void SPAWM_RECTANGLE::testpub_loopCB(const ros::TimerEvent&)
{
    // geometry_msgs::Point test_point;//25*6
	// for(int y=0;y<=60;y++)
	// {
	// 	for(int x=0;x<=250;x++)
	// 	{
	// 		test_point.x=1.0*x/10;
	// 		test_point.y=1.0*y/10;
	// 		get_Occupancy(test_point);
	// 	}
	// }
	// points.points.clear();
}
void SPAWM_RECTANGLE::init_spawm()
{	
	up_left_peak.position.x=start_pose_x;
	up_left_peak.position.y=start_pose_y;
	up_right_peak.position.x=start_pose_x;
	up_right_peak.position.y=start_pose_y;
	down_left_peak.position.x=start_pose_x;
	down_left_peak.position.y=start_pose_y;
	down_right_peak.position.x=start_pose_x;
	down_right_peak.position.y=start_pose_y;

	edges.poses.push_back(up_left_peak);

}
void SPAWM_RECTANGLE::spawm_loopCB(const ros::TimerEvent&)
{
	if(map_receive_flag)
	{
		geometry_msgs::PoseArray last_edges=edges;
		edges.poses.clear();
		//scan peaks
		if(scan_up||scan_left)
		{
			if(scan_up_left)
			{
				up_left_peak.position.x-=rcd*step;
				up_left_peak.position.y+=rcd*step;
				if(get_Occupancy(up_left_peak.position)>0)
				{
					scan_up_left=false;
					ROS_WARN("-touch up left boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_left_peak);
					edges.poses.push_back(up_left_peak);
				}
			}
			else if(scan_up)
			{
				up_left_peak.position.y+=rcd*step;
				if(get_Occupancy(up_left_peak.position)>0)
				{
					scan_up=false;
					ROS_WARN("-touch up boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_left_peak);
					edges.poses.push_back(up_left_peak);
				}
			}
			else if(scan_left)
			{
				up_left_peak.position.x-=rcd*step;
				if(get_Occupancy(up_left_peak.position)>0)
				{
					scan_left=false;
					ROS_WARN("-touch left boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_left_peak);
					edges.poses.push_back(up_left_peak);
				}
			}
		}
		else
			edges.poses.push_back(up_left_peak);

		if(scan_up||scan_right)
		{
			if(scan_up_right)
			{
				up_right_peak.position.x+=rcd*step;
				up_right_peak.position.y+=rcd*step;
				if(get_Occupancy(up_right_peak.position)>0)
				{
					scan_up_right=false;
					ROS_WARN("touch up right boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_right_peak);
					edges.poses.push_back(up_right_peak);
				}
			}
			else if(scan_up)
			{
				up_right_peak.position.y+=rcd*step;
				if(get_Occupancy(up_right_peak.position)>0)
				{
					scan_up=false;
					ROS_WARN("-touch up boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_right_peak);
					edges.poses.push_back(up_right_peak);
				}
			}
			else if(scan_right)
			{
				up_right_peak.position.x+=rcd*step;
				if(get_Occupancy(up_right_peak.position)>0)
				{
					scan_right=false;
					ROS_WARN("-touch  right boundary ");
				}
				else
				{
					last_edges.poses.push_back(up_right_peak);
					edges.poses.push_back(up_right_peak);
				}
			}

		}
		else
			edges.poses.push_back(up_right_peak);

		if(scan_down||scan_left)
		{
			if(scan_down_left)
			{
				down_left_peak.position.x-=rcd*step;
				down_left_peak.position.y-=rcd*step;
				if(get_Occupancy(down_left_peak.position)>0)
				{
					scan_down_left=false;
					ROS_WARN("-touch down left boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_left_peak);
					edges.poses.push_back(down_left_peak);
				}
			}
			else if(scan_down)
			{
				down_left_peak.position.y-=rcd*step;
				if(get_Occupancy(down_left_peak.position)>0)
				{
					scan_down=false;
					ROS_WARN("-touch down boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_left_peak);
					edges.poses.push_back(down_left_peak);
				}
			}
			else if(scan_left)
			{
				down_left_peak.position.x-=rcd*step;
				if(get_Occupancy(down_left_peak.position)>0)
				{
					scan_left=false;
					ROS_WARN("-touch left boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_left_peak);
					edges.poses.push_back(down_left_peak);
				}
			}
		}
		else
			edges.poses.push_back(down_left_peak);
		if(scan_down||scan_right)
		{
			if(scan_down_right)
			{
				down_right_peak.position.x+=rcd*step;
				down_right_peak.position.y-=rcd*step;
				if(get_Occupancy(down_right_peak.position)>0)
				{
					scan_down_right=false;
					ROS_WARN("-touch down right boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_right_peak);
					edges.poses.push_back(down_right_peak);
				}
			}
			else if(scan_down)
			{
				down_right_peak.position.y-=rcd*step;
				if(get_Occupancy(down_right_peak.position)>0)
				{
					scan_down=false;
					ROS_WARN("-touch down boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_right_peak);
					edges.poses.push_back(down_right_peak);
				}
			}
			else if(scan_right)
			{
				down_right_peak.position.x+=rcd*step;
				if(get_Occupancy(down_right_peak.position)>0)
				{
					scan_right=false;
					ROS_WARN("-touch right boundary ");
				}
				else
				{
					last_edges.poses.push_back(down_right_peak);
					edges.poses.push_back(down_right_peak);
				}
			}

		}
		else
			edges.poses.push_back(down_right_peak);

		//check boundary
			//check up boundary
		geometry_msgs::PoseArray up_edge=get_poses_between_peaks(up_left_peak,up_right_peak);
		if(scan_up)
		{
			for(geometry_msgs::Pose each_edge_pose:up_edge.poses)
			{
				if(get_Occupancy(each_edge_pose)>0)
				{
					scan_up=false;
					scan_up_left=false;
					scan_up_right=false;
					//ROS_INFO("boundary:%.2f,%.2f",each_edge_pose.position.x,each_edge_pose.position.y);
				}
			}
			if(!scan_up)
			{
				up_left_peak.position.y-=rcd*step;
				up_right_peak.position.y-=rcd*step;
				ROS_WARN("touch up boundary");
			}
		}
		if(scan_up)
		{
			edges.poses.insert(edges.poses.end(),up_edge.poses.begin(),up_edge.poses.end());
		}
			//check left boundary
		geometry_msgs::PoseArray left_edge=get_poses_between_peaks(up_left_peak,down_left_peak);
		if(scan_left)
		{
			for(geometry_msgs::Pose each_edge_pose:left_edge.poses)
			{
				if(get_Occupancy(each_edge_pose)>0)
				{
					scan_left=false;
					scan_up_left=false;
					scan_down_left=false;
				}
			}
			if(!scan_left)
			{
				up_left_peak.position.x+=rcd*step;
				down_left_peak.position.x+=rcd*step;
				ROS_WARN("touch left boundary ");
			}
		}
		if(scan_left)
		{
			edges.poses.insert(edges.poses.end(),left_edge.poses.begin(),left_edge.poses.end());
		}
			//check dowm boundary
		geometry_msgs::PoseArray down_edge=get_poses_between_peaks(down_right_peak,down_left_peak);
		if(scan_down)
		{
			for(geometry_msgs::Pose each_edge_pose:down_edge.poses)
			{
				if(get_Occupancy(each_edge_pose)>0)
				{
					scan_down=false;
					scan_down_left=false;
					scan_down_right=false;
				}
			}
			if(!scan_down)
			{
				down_right_peak.position.y+=rcd*step;
				down_left_peak.position.y+=rcd*step;		
				ROS_WARN("touch down boundary ");		
			}
		}
		if(scan_down)
		{
			edges.poses.insert(edges.poses.end(),down_edge.poses.begin(),down_edge.poses.end());
		}
			//check right boundary
		geometry_msgs::PoseArray right_edge=get_poses_between_peaks(up_right_peak,down_right_peak);
		if(scan_right)
		{
			for(geometry_msgs::Pose each_edge_pose:right_edge.poses)
			{
				if(get_Occupancy(each_edge_pose)>0)
				{
					scan_right=false;
					scan_down_right=false;
					scan_up_right=false;
					//ROS_INFO("right boundary:%.2f,%.2f",each_edge_pose.position.x,each_edge_pose.position.y);
				}
			}
			if(!scan_right)
			{
				down_right_peak.position.x-=rcd*step;
				up_right_peak.position.x-=rcd*step;		
				ROS_WARN("touch right boundary");	
			}
		}
		if(scan_right)
		{
			edges.poses.insert(edges.poses.end(),right_edge.poses.begin(),right_edge.poses.end());
		}


		points.points.clear();
		draw_marker(edges);
	}
}

geometry_msgs::PoseArray SPAWM_RECTANGLE::get_poses_between_peaks(geometry_msgs::Pose& peak1,geometry_msgs::Pose& peak2)
{
	geometry_msgs::PoseArray edge;
	int counter=0;
	//ROS_INFO("x dist=%f,y dist=%f",peak1.position.x-peak2.position.x,peak1.position.y-peak2.position.y);
	if(peak1.position.x==peak2.position.x&&peak1.position.y==peak2.position.y)
	{
		edge.poses.push_back(peak1);
		return edge;
	}
	if(peak1.position.x==peak2.position.x)
	{
		//ROS_INFO("left right edge scan");
		//compare y
		if(peak1.position.y>peak2.position.y)
		{
			for(float y=peak2.position.y+step*rcd;y<peak1.position.y-step*rcd;y+=step*rcd)
			{
				geometry_msgs::Pose pose_;
				pose_.position.x=peak1.position.x;
				pose_.position.y=y;
				edge.poses.push_back(pose_);
				counter++;
			}
		}
		else
		{
			for(float y=peak2.position.y-step*rcd;y>peak1.position.y+step*rcd;y-=step*rcd)
			{
				geometry_msgs::Pose pose_;
				pose_.position.x=peak1.position.x;
				pose_.position.y=y;
				edge.poses.push_back(pose_);
				counter++;
			}
		}
	}
	if(peak1.position.y==peak2.position.y)
	{
		//ROS_INFO("up down edge scan");
		//compare x
		if(peak1.position.x>peak2.position.x)
		{
			for(float x=peak2.position.x+step*rcd;x<peak1.position.x-step*rcd;x+=step*rcd)
			{
				geometry_msgs::Pose pose_;
				pose_.position.x=x;
				pose_.position.y=peak1.position.y;
				edge.poses.push_back(pose_);
				counter++;
			}
		}
		else
		{
			for(float x=peak2.position.x-step*rcd;x>peak1.position.x+step*rcd;x-=step*rcd)
			{
				geometry_msgs::Pose pose_;
				pose_.position.x=x;
				pose_.position.y=peak1.position.y;
				edge.poses.push_back(pose_);
				counter++;
			}
		}
	}
	//ROS_INFO("add %d edge points",counter);
	return edge;
}

geometry_msgs::PoseArray SPAWM_RECTANGLE::spawm()
{  
	//geometry_msgs::Pose up_left_peak;
	// geometry_msgs::Pose up_right_peak;
	// geometry_msgs::Pose down_left_peak;
	// geometry_msgs::Pose down_right_peak;

	// up_left_peak.position.x=start_pose_x;
	// up_left_peak.position.y=start_pose_y;
	// up_right_peak.position.x=start_pose_x;
	// up_right_peak.position.y=start_pose_y;
	// down_left_peak.position.x=start_pose_x;
	// down_left_peak.position.y=start_pose_y;
	// down_right_peak.position.x=start_pose_x;
	// down_right_peak.position.y=start_pose_y;

	// geometry_msgs::PoseArray edges;
	// edges.poses.push_back(up_left_peak);

	// //expand
	// //int threshold=60000;
	// int i=0;
	// bool scan_up||scan_left=true;
	// bool scan_up||scan_right=true;
	// bool scan_down||scan_left=true;
	// bool scan_down||scan_right=true;
	// while(i<20)
	// {	
		// geometry_msgs::PoseArray last_edges=edges;
		// edges.poses.clear();
		// //scan peaks
		// if(scan_up||scan_left)
		// {
		// 	up_left_peak.position.x-=rcd*step;
		// 	if(get_Occupancy(down_right_peak.position)>0)
		// 	{
		// 		scan_down||scan_right=false;
		// 		ROS_WARN("touch down right boundary ");
		// 	}
		// 	else
		// 	{
		// 		edges.poses.push_back(down_right_peak);
		// 		ROS_INFO("add new peak at:[%.1f,%.1f] ",down_right_peak.position.x,down_right_peak.position.y);
		// 	}
		// }
		// else
		// 	edges.poses.push_back(down_right_peak);

		//scan by edges
		// for(geometry_msgs::Pose each_edge_pose:last_edges.poses)
		// {
		// 	if((each_edge_pose.position.x-start_pose_x>0)&&get_Occupancy(each_edge_pose.position.x+rcd,each_edge_pose.position.y)>0)
		// 	{
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x+rcd;
		// 		n_pose.position.y=each_edge_pose.position.y;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.x-start_pose_x<0)&&get_Occupancy(each_edge_pose.position.x-rcd,each_edge_pose.position.y)>0)
		// 	{
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x-rcd;
		// 		n_pose.position.y=each_edge_pose.position.y;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.y-start_pose_y>0)&&get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y+rcd)>0)
		// 	{
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x;
		// 		n_pose.position.y=each_edge_pose.position.y+rcd;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.y-start_pose_y<0)&&get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y)>0)
		// 	{
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x;
		// 		n_pose.position.y=each_edge_pose.position.y-rcd;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// }



		//scan by edges
		// for(geometry_msgs::Pose each_edge_pose:last_edges.poses)
		// {
		// 	if((each_edge_pose.position.y==up_left_peak.position.y)&&(!touch_up_boundary))
		// 	{
		// 		if(get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y+rcd)>0)
		// 			touch_up_boundary=true;
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x;
		// 		n_pose.position.y=each_edge_pose.position.y+rcd;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.x==up_left_peak.position.x)&&(!scan_left))
		// 	{
		// 		if(get_Occupancy(each_edge_pose.position.x-rcd,each_edge_pose.position.y)>0)
		// 			scan_left=true;
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x-rcd;
		// 		n_pose.position.y=each_edge_pose.position.y;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.y==down_left_peak.position.y)&&(!scan_down))
		// 	{
		// 		if(get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y-rcd)>0)
		// 			scan_down=true;
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x;
		// 		n_pose.position.y=each_edge_pose.position.y-rcd;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// 	if((each_edge_pose.position.x==down_right_peak.position.x)&&(!scan_right))
		// 	{
		// 		if(get_Occupancy(each_edge_pose.position.x+rcd,each_edge_pose.position.y)>0)
		// 			scan_right=true;
		// 		geometry_msgs::Pose n_pose;
		// 		n_pose.position.x=each_edge_pose.position.x+rcd;
		// 		n_pose.position.y=each_edge_pose.position.y;
		// 		edges.poses.push_back(n_pose);
		// 	}
		// }
		
		//ROS_INFO("edge size:%d",edges.poses.size());
	// 	points.points.clear();
	// 	draw_marker(edges);
	// 	i++;
	// }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "spawm_rectangle");
	SPAWM_RECTANGLE spawm;
	ros::spin();
    return 0;
}