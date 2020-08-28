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
#include "walk_in_rectangle/Rectangle.h"

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

	ros::Publisher rect_pub,marker_pub,map_pub,center_pose_pub;
    ros::Timer timer;
    void testpub_loopCB(const ros::TimerEvent&);
	//This is the main loop,searching for the rectangles in the map and publish them
	void spawm_loopCB(const ros::TimerEvent&);
	ros::Subscriber map_sub;
	void mapCB(const nav_msgs::OccupancyGrid& map);
	//get Occupancy from map
	int8_t get_Occupancy(const geometry_msgs::Point& position);
	int8_t get_Occupancy(const geometry_msgs::Pose& pose);
	int8_t get_Occupancy(const float& x,const float& y);
	//set Occupancy to the map_(copy)
	void set_Occupancy(const float& x,const float& y,const int8_t& ocpc_);
	void set_Occupancy(const geometry_msgs::PoseArray& pose_array_,const int8_t& ocpc_);
	void set_Occupancy(const walk_in_rectangle::Rectangle& rect_,const int8_t& ocpc_);
	int get_index(const float& x_,const float& y_);
	void init_marker();
	void init_rectangle(const double& start_x_,const double& start_y_);
	//visualization
	visualization_msgs::Marker points, line_strip, goal_circle;
	void draw_marker(const geometry_msgs::Point& position);
	void draw_marker(const geometry_msgs::Pose& pose);
	void draw_marker(const geometry_msgs::PoseArray& pose_array);
	void draw_marker(const walk_in_rectangle::Rectangle& rect_);
	//expand rectangle ince from the given position
	geometry_msgs::PoseArray expand_rectangle();
	//get points between two peaks
	geometry_msgs::PoseArray get_poses_between_peaks(geometry_msgs::Pose& peak1,geometry_msgs::Pose& peak2);
	//stash the rectangle that explored
	void save_rect( const walk_in_rectangle::Rectangle& rect_buffer_);

	//the peaks of a rectangle
	geometry_msgs::Pose up_left_peak;
	geometry_msgs::Pose up_right_peak;
	geometry_msgs::Pose down_left_peak;
	geometry_msgs::Pose down_right_peak;
	///the edges of a rectangle
	geometry_msgs::PoseArray edges;
	//the gateway(unoccupied edge point)
	geometry_msgs::PoseArray left_gateway,right_gateway,up_gateway,down_gateway;
	//rectangles
	geometry_msgs::PoseArray first_rect,second_rect,process_data_buffer;
	//unoccupied edge point would become a seed,new rectangle grows from here
	geometry_msgs::PoseArray seed_stack;//FIFO
	//rectangle array
	std::vector<walk_in_rectangle::Rectangle> rectangle_array;
	walk_in_rectangle::Rectangle rect_buffer;

	//flags
	bool map_receive_flag=false;
	bool scan_up_left=true;
	bool scan_down_left=true;
	bool scan_up_right=true;
	bool scan_down_right=true;
	bool scan_up=true;
	bool scan_down=true;
	bool scan_left=true;
	bool scan_right=true;
	bool first_prepare_flag=true;
	bool rect_expand_finish_flag=false;
	bool occupied_flag=false;
	//params
	double step;
	double seed_x,seed_y;
	//status of the main loop
	enum STATUS {PREPARE,PROCESS,DONE} status;
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

	map_sub=n.subscribe("/map", 1, &SPAWM_RECTANGLE::mapCB, this);//nav_msgs/OccupancyGrid
	map_pub=n.advertise<nav_msgs::OccupancyGrid>("/N_map",10);
	center_pose_pub=n.advertise<geometry_msgs::PoseArray>("/center_pose_array",10);
	rect_pub=n.advertise<walk_in_rectangle::Rectangle>("/rect_spawm",10);
	marker_pub = n.advertise<visualization_msgs::Marker>("/Marker", 10);

	status=PREPARE;
	timer = n.createTimer(ros::Duration((1.0)/200), &SPAWM_RECTANGLE::spawm_loopCB, this);
	//timer = n.createTimer(ros::Duration((1.0)/0.5), &SPAWM_RECTANGLE::testpub_loopCB, this);

	init_marker();

	// ros::Duration(1.0).sleep();

}
void SPAWM_RECTANGLE::mapCB(const nav_msgs::OccupancyGrid& map)
{	
	map_=map;
	//map_.info.resolution=0.05;
	map_receive_flag=true;
	ROS_INFO("map received");
	map_pub.publish(map_);
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
	points.scale.x = 0.1;
	points.scale.y = 0.1;

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
	{
		ROS_ERROR("index WARN,[%.2f,%.2f]",x,y);
	}
		
		
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
void SPAWM_RECTANGLE::set_Occupancy(const float& x,const float& y,const int8_t& ocpc_)
{
	int8_t Occupancy=-1;
	if(map_.info.resolution==0)
	{
		ROS_WARN("map_.info.resolution=%.3f,unknow Occupancy",map_.info.resolution);
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
		map_.data[index]=ocpc_;
		// map_.data[index+1]=ocpc_;
		// map_.data[index-1]=ocpc_;
		// map_.data[index+col]=ocpc_;
		// map_.data[index-col]=ocpc_;
		// map_.data[index+1+col]=ocpc_;
		// map_.data[index-1+col]=ocpc_;
		// map_.data[index+1-col]=ocpc_;
		// map_.data[index-1-col]=ocpc_;
		
		//ROS_INFO("index:%d,Occupancy:%d",index,Occupancy);
	}
	else
		ROS_ERROR("index WARN");
		
}
void SPAWM_RECTANGLE::set_Occupancy(const geometry_msgs::PoseArray& pose_array_,const int8_t& ocpc_)
{
	if(map_.info.resolution==0)
	{
		ROS_WARN("map_.info.resolution=%.3f,unknow Occupancy",map_.info.resolution);
	}

	for(geometry_msgs::Pose each_pose:pose_array_.poses)
	{
		float x=each_pose.position.x;
		float y=each_pose.position.y;
		int index;
		int row=(fabs(y-map_data_origion_y)/map_.info.resolution);//像素行数
		int col=map_.info.width;//像素列数
		int remain=(fabs(x-map_data_origion_x)/map_.info.resolution);
		index=(int)(row*col+remain);
		if(index<=(map_.info.width*map_.info.height))
		{
			map_.data[index]=ocpc_;
			// map_.data[index+1]=ocpc_;
			// map_.data[index-1]=ocpc_;
			// map_.data[index+col]=ocpc_;
			// map_.data[index-col]=ocpc_;
			// map_.data[index+1+col]=ocpc_;
			// map_.data[index-1+col]=ocpc_;
			// map_.data[index+1-col]=ocpc_;
			// map_.data[index-1-col]=ocpc_;
		}
		else
			ROS_ERROR("index WARN");
	}
	map_pub.publish(map_);	
}
int SPAWM_RECTANGLE::get_index(const float& x_,const float& y_)
{
	float x= x_;
	float y= y_;
	int index;
	int row=(fabs(y-map_data_origion_y)/map_.info.resolution);//像素行数
	int col=map_.info.width;//像素列数
	int remain=(fabs(x-map_data_origion_x)/map_.info.resolution);
	index=(int)(row*col+remain);
	return index;
}
void SPAWM_RECTANGLE::set_Occupancy( const walk_in_rectangle::Rectangle& rect_,const int8_t& ocpc_)
{
	int index_U_L=get_index(rect_.U_L.x,rect_.U_L.y);
	int index_U_R=get_index(rect_.U_R.x,rect_.U_R.y);
	int index_D_L=get_index(rect_.D_L.x,rect_.D_L.y);
	int index_D_R=get_index(rect_.D_R.x,rect_.D_R.y);
	//ROS_INFO("%d %d %d %d",index_U_L,index_U_R,index_D_L,index_D_R);

	for(int i=0;i<=abs(index_U_R-index_D_R)/map_.info.width;i++)
	{
		for(int j=0;j<=abs( index_D_L-index_D_R);j++)
		{
			map_.data[map_.info.width*i+j+index_D_L]=ocpc_;
		}
	}

	map_pub.publish(map_);	
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
void SPAWM_RECTANGLE::draw_marker(const walk_in_rectangle::Rectangle& rect_)
{
	line_strip.points.clear();
	line_strip.points.push_back(rect_.U_R);
	line_strip.points.push_back(rect_.D_R);
	line_strip.points.push_back(rect_.D_L);
	line_strip.points.push_back(rect_.U_L);
	line_strip.points.push_back(rect_.U_R);

	marker_pub.publish(line_strip);
}
void SPAWM_RECTANGLE::testpub_loopCB(const ros::TimerEvent&)
{
    // there is nothing
}
void SPAWM_RECTANGLE::spawm_loopCB(const ros::TimerEvent&)
{//main loop
	if(map_receive_flag)//make sure that the map is received
	{
		if(status==PREPARE)
		{
			if(first_prepare_flag)
			{//init the first seed,first rectangle would grows from here
				seed_x=start_pose_x;
				seed_y=start_pose_y;
				init_rectangle(seed_x,seed_y);
				status=PROCESS;
				first_prepare_flag=false;
				ROS_INFO("init first rect");
			}
			else
			{
				//save rectangle && publish it

				//get edge && set edge occupied
				set_Occupancy(rect_buffer,20);
				//stash new seed list
				if(!occupied_flag)
					seed_stack.poses.insert(seed_stack.poses.end(),process_data_buffer.poses.begin(),process_data_buffer.poses.end());
				//check if stack empty
				if(seed_stack.poses.empty())
				{
					ROS_WARN("seed_stack is empty");
					status=DONE;
				}
				else
				{	//get next seed from stack button && pop first seed && check if occupied 
					seed_x=seed_stack.poses.front().position.x;
					seed_y=seed_stack.poses.front().position.y;
					ROS_INFO("seed:[%.2f,%.2f]",seed_x,seed_y);
					seed_stack.poses.erase(seed_stack.poses.begin());
					if(get_Occupancy(seed_x,seed_y)!=0)
					{
						ROS_WARN("seed occupied,check next");
						occupied_flag=true;
					}
					else
					{
						//init process
						init_rectangle(seed_x,seed_y);		
						//set status to process
						status=PROCESS;
						occupied_flag=false;
						
				save_rect(rect_buffer);
				rect_pub.publish(rect_buffer);

			// geometry_msgs::PoseArray center_points_array;
			// for(int i=0;i<rectangle_array.size();i++)
			// {
			// 	geometry_msgs::Pose center_pose;
			// 	if(rectangle_array[i].U_R.x-rectangle_array[i].U_L.x<rcd/2)
			// 		continue;
			// 	if(rectangle_array[i].U_R.y-rectangle_array[i].D_L.y<rcd/2)
			// 		continue;	
			// 	center_pose.position.x=(rectangle_array[i].U_L.x+rectangle_array[i].D_R.x)/2;
			// 	center_pose.position.y=(rectangle_array[i].U_L.y+rectangle_array[i].D_R.y)/2;
			// 	center_points_array.poses.push_back(center_pose);
			// }
			// //publish center points
			// center_pose_pub.publish(center_points_array);
			// //vistualization
			// points.points.clear();
			// draw_marker(center_points_array);

					}
				}
			}
		}
		else if(status==PROCESS)
		{
			//process
			process_data_buffer=expand_rectangle();
			//check if finished
			if(rect_expand_finish_flag)
			{
				ROS_INFO("spawm once");
				//set status to prepare
				status=PREPARE;
			}
		}
		else if(status==DONE)
		{
			ROS_INFO("calculate center points");
			// calculate center points of rectangles
			// --input:		rectangle_array
			// --output:	center_points_array
			geometry_msgs::PoseArray center_points_array;
			for(int i=0;i<rectangle_array.size();i++)
			{
				geometry_msgs::Pose center_pose;
				if(rectangle_array[i].U_R.x-rectangle_array[i].U_L.x<rcd)
					continue;
				if(rectangle_array[i].U_R.y-rectangle_array[i].D_L.y<rcd)
					continue;	
				center_pose.position.x=(rectangle_array[i].U_L.x+rectangle_array[i].D_R.x)/2;
				center_pose.position.y=(rectangle_array[i].U_L.y+rectangle_array[i].D_R.y)/2;
				center_points_array.poses.push_back(center_pose);
			}
			//publish center points
			center_pose_pub.publish(center_points_array);
			//vistualization
			points.points.clear();
			draw_marker(center_points_array);
			ROS_INFO("\n--------------------------\n     spawm rectangle succeed \n--------------------------");
			//stop timer
			timer.stop();
		}				
	}
}
void SPAWM_RECTANGLE::save_rect(const walk_in_rectangle::Rectangle& rect_buffer_)
{
	walk_in_rectangle::Rectangle rect_=rect_buffer_;
	ROS_INFO("\n rect_.U_R:[%.2f,%.2f]\n rect_.D_R:[%.2f,%.2f]\n rect_.D_L:[%.2f,%.2f]\n rect_.U_L:[%.2f,%.2f]",
					rect_.U_R.x,rect_.U_R.y,rect_.D_R.x,rect_.D_R.y,rect_.D_L.x,rect_.D_L.y,rect_.U_L.x,rect_.U_L.y);
	rectangle_array.push_back(rect_);
	//draw_marker(rect_);
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
void SPAWM_RECTANGLE::init_rectangle(const double& start_x_,const double& start_y_)
{
	up_left_peak.position.x=start_x_;
	up_left_peak.position.y=start_y_;
	up_right_peak.position.x=start_x_;
	up_right_peak.position.y=start_y_;
	down_left_peak.position.x=start_x_;
	down_left_peak.position.y=start_y_;
	down_right_peak.position.x=start_x_;
	down_right_peak.position.y=start_y_;

	scan_up_left=true;
	scan_up_right=true;
	scan_down_left=true;
	scan_down_right=true;
	scan_down=true;
	scan_up=true;
	scan_right=true;
	scan_left=true;
	rect_expand_finish_flag=false;

	edges.poses.clear();
	edges.poses.push_back(up_left_peak);
	ROS_INFO("init new rect expand");
}
geometry_msgs::PoseArray SPAWM_RECTANGLE::expand_rectangle()
{  
	geometry_msgs::PoseArray gateway;
	//geometry_msgs::PoseArray last_edges=edges;
	edges.poses.clear();
	//scan peaks
	if(scan_up||scan_left)
	{
		if(scan_up_left)
		{
			if(get_Occupancy(up_left_peak.position.x-=rcd*step,up_left_peak.position.y+=rcd*step)!=0)
			{
				up_left_peak.position.x+=rcd*step;
				up_left_peak.position.y-=rcd*step;
				scan_up_left=false;
				ROS_WARN("-touch up left boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_left_peak);
				edges.poses.push_back(up_left_peak);
			}
		}
		else if(scan_up)
		{
			if(get_Occupancy(up_left_peak.position.x,up_left_peak.position.y+=rcd*step)!=0)
			{
				up_left_peak.position.y-=rcd*step;
				scan_up=false;
				ROS_WARN("-touch up boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_left_peak);
				edges.poses.push_back(up_left_peak);
			}
		}
		else if(scan_left)
		{
			if(get_Occupancy(up_left_peak.position.x-=rcd*step,up_left_peak.position.y)!=0)
			{
				up_left_peak.position.x+=rcd*step;
				scan_left=false;
				ROS_WARN("-touch left boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_left_peak);
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
			if(get_Occupancy(up_right_peak.position.x+=rcd*step,up_right_peak.position.y+=rcd*step)!=0)
			{
				up_right_peak.position.x-=rcd*step;
				up_right_peak.position.y-=rcd*step;
				scan_up_right=false;
				ROS_WARN("touch up right boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_right_peak);
				edges.poses.push_back(up_right_peak);
			}
		}
		else if(scan_up)
		{
			if(get_Occupancy(up_right_peak.position.x,up_right_peak.position.y+=rcd*step)!=0)
			{
				up_right_peak.position.y-=rcd*step;
				scan_up=false;
				ROS_WARN("-touch up boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_right_peak);
				edges.poses.push_back(up_right_peak);
			}
		}
		else if(scan_right)
		{
			if(get_Occupancy(up_right_peak.position.x+=rcd*step,up_right_peak.position.y)!=0)
			{
				up_right_peak.position.x-=rcd*step;
				scan_right=false;
				ROS_WARN("-touch  right boundary ");
			}
			else
			{
				//last_edges.poses.push_back(up_right_peak);
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
			if(get_Occupancy(down_left_peak.position.x-=rcd*step,down_left_peak.position.y-=rcd*step)!=0)
			{
				down_left_peak.position.x+=rcd*step;
				down_left_peak.position.y+=rcd*step;
				scan_down_left=false;
				ROS_WARN("-touch down left boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_left_peak);
				edges.poses.push_back(down_left_peak);
			}
		}
		else if(scan_down)
		{
			if(get_Occupancy(down_left_peak.position.x,down_left_peak.position.y-=rcd*step)!=0)
			{
				down_left_peak.position.y+=rcd*step;
				scan_down=false;
				ROS_WARN("-touch down boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_left_peak);
				edges.poses.push_back(down_left_peak);
			}
		}
		else if(scan_left)
		{
			down_left_peak.position.x-=rcd*step;
			if(get_Occupancy(down_left_peak.position.x-=rcd*step,down_left_peak.position.y)!=0)
			{
				scan_left=false;
				ROS_WARN("-touch left boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_left_peak);
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
			if(get_Occupancy(down_right_peak.position.x+=rcd*step,down_right_peak.position.y-=rcd*step)!=0)
			{
				down_right_peak.position.x-=rcd*step;
				down_right_peak.position.y+=rcd*step;
				scan_down_right=false;
				ROS_WARN("-touch down right boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_right_peak);
				edges.poses.push_back(down_right_peak);
			}
		}
		else if(scan_down)
		{
			if(get_Occupancy(down_right_peak.position.x,down_right_peak.position.y-=rcd*step)!=0)
			{
				down_right_peak.position.y+=rcd*step;
				scan_down=false;
				ROS_WARN("-touch down boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_right_peak);
				edges.poses.push_back(down_right_peak);
			}
		}
		else if(scan_right)
		{
			if(get_Occupancy(down_right_peak.position.x+=rcd*step,down_right_peak.position.y)!=0)
			{
				down_right_peak.position.x-=rcd*step;
				scan_right=false;
				ROS_WARN("-touch right boundary ");
			}
			else
			{
				//last_edges.poses.push_back(down_right_peak);
				edges.poses.push_back(down_right_peak);
			}
		}

	}
	else
		edges.poses.push_back(down_right_peak);
	
	//check peak
	if(up_left_peak.position.y>up_right_peak.position.y)
		up_left_peak.position.y=up_right_peak.position.y;
	if(up_left_peak.position.y<up_right_peak.position.y)
		up_right_peak.position.y=up_left_peak.position.y;
	if(up_left_peak.position.x>down_left_peak.position.x)
		down_left_peak.position.x=up_left_peak.position.x;
	if(up_left_peak.position.x<down_left_peak.position.x)
		up_left_peak.position.x=down_left_peak.position.x;
	if(down_right_peak.position.y>down_left_peak.position.y)
		down_left_peak.position.y=down_right_peak.position.y;
	if(down_right_peak.position.y<down_left_peak.position.y)
		down_right_peak.position.y=down_left_peak.position.y;
	if(up_right_peak.position.x>down_right_peak.position.x)
		up_right_peak.position.x=down_right_peak.position.x;
	if(up_right_peak.position.x<down_right_peak.position.x)
		down_right_peak.position.x=up_right_peak.position.x;

	//check boundary
		//check up boundary
	geometry_msgs::PoseArray up_edge=get_poses_between_peaks(up_left_peak,up_right_peak);
	up_gateway.poses.clear();
	for(geometry_msgs::Pose each_edge_pose:up_edge.poses)
	{
		if(get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y+rcd*step)==0)
		{
			geometry_msgs::Pose pose_;
			pose_.position.x=each_edge_pose.position.x;
			pose_.position.y=each_edge_pose.position.y+map_.info.resolution;
			up_gateway.poses.push_back(pose_);
		}
	}
	if(scan_up)
	{
		for(geometry_msgs::Pose each_edge_pose:up_edge.poses)
		{
			if(get_Occupancy(each_edge_pose)!=0)
			{
				scan_up=false;
				scan_up_left=false;
				scan_up_right=false;
			}
		}
		if(!scan_up)
		{
			for(int i=0;i<up_edge.poses.size();i++)//geometry_msgs::Pose each_edge_pose:up_edge.poses)
			{
				up_edge.poses[i].position.y-=rcd*step;
			}
			up_left_peak.position.y-=rcd*step;
			up_right_peak.position.y-=rcd*step;
			ROS_WARN("touch up boundary");
		}
	}
	edges.poses.insert(edges.poses.end(),up_edge.poses.begin(),up_edge.poses.end());

		//check left boundary
	geometry_msgs::PoseArray left_edge=get_poses_between_peaks(up_left_peak,down_left_peak);
	left_gateway.poses.clear();
	for(geometry_msgs::Pose each_edge_pose:left_edge.poses)
	{
		if(get_Occupancy(each_edge_pose.position.x-rcd*step,each_edge_pose.position.y)==0)
		{
			geometry_msgs::Pose pose_;
			pose_.position.x=each_edge_pose.position.x-map_.info.resolution;
			pose_.position.y=each_edge_pose.position.y;
			left_gateway.poses.push_back(pose_);
		}
	}
	if(scan_left)
	{
		for(geometry_msgs::Pose each_edge_pose:left_edge.poses)
		{
			if(get_Occupancy(each_edge_pose)!=0)
			{
				scan_left=false;
				scan_up_left=false;
				scan_down_left=false;
			}
		}
		if(!scan_left)
		{
			for(int i=0;i<left_edge.poses.size();i++)
			{
				left_edge.poses[i].position.x+=rcd*step;
			}
			up_left_peak.position.x+=rcd*step;
			down_left_peak.position.x+=rcd*step;
			ROS_WARN("touch left boundary ");
		}
	}
	edges.poses.insert(edges.poses.end(),left_edge.poses.begin(),left_edge.poses.end());
		//check dowm boundary
	geometry_msgs::PoseArray down_edge=get_poses_between_peaks(down_right_peak,down_left_peak);
	down_gateway.poses.clear();
	for(geometry_msgs::Pose each_edge_pose:down_edge.poses)
	{
		if(get_Occupancy(each_edge_pose.position.x,each_edge_pose.position.y-rcd*step)==0)
		{
			geometry_msgs::Pose pose_;
			pose_.position.x=each_edge_pose.position.x;
			pose_.position.y=each_edge_pose.position.y-map_.info.resolution;
			left_gateway.poses.push_back(pose_);
		}
	}
	if(scan_down)
	{
		for(geometry_msgs::Pose each_edge_pose:down_edge.poses)
		{
			if(get_Occupancy(each_edge_pose)!=0)
			{
				scan_down=false;
				scan_down_left=false;
				scan_down_right=false;
			}
		}
		if(!scan_down)
		{
			for(int i=0;i<down_edge.poses.size();i++)
			{
				down_edge.poses[i].position.y+=rcd*step;
			}
			down_right_peak.position.y+=rcd*step;
			down_left_peak.position.y+=rcd*step;		
			ROS_WARN("touch down boundary ");		
		}
	}
	edges.poses.insert(edges.poses.end(),down_edge.poses.begin(),down_edge.poses.end());

		//check right boundary
	geometry_msgs::PoseArray right_edge=get_poses_between_peaks(up_right_peak,down_right_peak);
	right_gateway.poses.clear();
	for(geometry_msgs::Pose each_edge_pose:right_edge.poses)
	{
		if(get_Occupancy(each_edge_pose.position.x+rcd*step,each_edge_pose.position.y)==0)
		{
			geometry_msgs::Pose pose_;
			pose_.position.x=each_edge_pose.position.x+map_.info.resolution;
			pose_.position.y=each_edge_pose.position.y;
			right_gateway.poses.push_back(pose_);
		}
	}
	if(scan_right)
	{
		for(geometry_msgs::Pose each_edge_pose:right_edge.poses)
		{
			if(get_Occupancy(each_edge_pose)!=0)
			{
				scan_right=false;
				scan_down_right=false;
				scan_up_right=false;
			}
		}
		if(!scan_right)
		{
			for(int i=0;i<right_edge.poses.size();i++)
			{
				right_edge.poses[i].position.x-=rcd*step;
			}
			down_right_peak.position.x-=rcd*step;
			up_right_peak.position.x-=rcd*step;		
			ROS_WARN("touch right boundary");	
		}
	}
	edges.poses.insert(edges.poses.end(),right_edge.poses.begin(),right_edge.poses.end());



	points.points.clear();
	//draw_marker(edges);

	gateway.poses.insert(gateway.poses.end(),up_gateway.poses.begin(),up_gateway.poses.end());
	gateway.poses.insert(gateway.poses.end(),left_gateway.poses.begin(),left_gateway.poses.end());
	gateway.poses.insert(gateway.poses.end(),down_gateway.poses.begin(),down_gateway.poses.end());
	gateway.poses.insert(gateway.poses.end(),right_gateway.poses.begin(),right_gateway.poses.end());

	//draw_marker(gateway);
	rect_buffer.U_L=up_left_peak.position;
	rect_buffer.D_L=down_left_peak.position;
	rect_buffer.D_R=down_right_peak.position;
	rect_buffer.U_R=up_right_peak.position;
	draw_marker(rect_buffer);
	if(!scan_down&&!scan_up&&!scan_right&&!scan_left)//done flag
	{
		rect_expand_finish_flag=true;
		ROS_WARN("rectangle expand succeed!");
	}
	else
	{
		rect_expand_finish_flag=false;
	}
	return gateway;
	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "spawm_rectangle");
	SPAWM_RECTANGLE spawm;
	ros::spin();
    return 0;
}