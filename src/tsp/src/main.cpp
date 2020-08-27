//==================================================================
// File			: main.cpp
// Author		: Rebecca Sagalyn
// Date			: Aug 25, 2013
// Description	: Driver for tsp.h
//==================================================================
#include <iostream>
#include <climits>
#include "tsp.h"
#include "usage.h"
#include "twoOpt.h"
#include "MyThread.h"		// thread wrapper class
//ros
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

// The length was annoying me.
#define CPS CLOCKS_PER_SEC

#define NUM_THREADS 1

ros::Publisher marker_pub;
// visualization_msgs::Marker points, line_strip, goal_circle;

void Center_arrayCB(const geometry_msgs::PoseArray& center_array_)
{
	// create tsp
	TSP tsp(center_array_);
	int n = tsp.get_size();
	// Start timing
	clock_t t = clock();
	clock_t t2;
	// Read cities 
	if (DEBUG)
		cout << "Reading cities" << endl;
	if(center_array_.poses.size()>1)
	{
		tsp.readCities(center_array_);
		tsp.printCities();

		//run tsp---------------->

		// Fill N x N matrix with distances between nodes
		if (DEBUG)
			cout << "\nFilling matrix" << endl;
		t2 = clock();
		tsp.fillMatrix_threads();
		if (DEBUG)
			cout << "Time to fill matrix: " << ((float) (clock() - t2)) / CPS
					<< " s\n";

		// Find a MST T in graph G
		if (DEBUG)
			cout << "\nFinding mst" << endl;
		t2 = clock();
		tsp.findMST_old();
		if (DEBUG)
			cout << "Time to find mst: " << ((float) (clock() - t2)) / CPS
					<< " s\n";

		// Find a minimum weighted matching M for odd vertices in T
		if (DEBUG)
			cout << "\nFinding perfect matching" << endl;
		t2 = clock();
		tsp.perfect_matching();
		if (DEBUG)
			cout << "Time to find matching: " << ((float) (clock() - t2)) / CPS
					<< " s\n\n";

		// Find the node that leads to the best path
		clock_t start, end;
		start = clock();

		// Create array of thread objects
		MyThread threads[NUM_THREADS];

		int best = INT_MAX;
		int bestIndex;
		int stop_here = NUM_THREADS;

		// Amount to increment starting node by each time
		int increment = 1; // by 1 if n < 1040

		if (n >= 600 && n < 1040)
			increment = 3;
		else if (n >= 1040 && n < 1800)
			increment = 8;
		else if (n >= 1800 && n < 3205)
			increment = 25; 		// ~ 220s @ 3200
		else if (n >= 3205 && n < 4005)
			increment = 50; 		// ~ 230s @ 4000
		else if (n >= 4005 && n < 5005)
			increment = 120;		// ~ 200 @ 5000
		else if (n >= 5005 && n < 6500)
			increment = 250;		// ~ 220s @ 6447
		else if (n >= 6500)
			increment = 500;

		int remaining = n;

		// Start at node zero
		int node = 0;

		// Count to get thread ids
		int count = 0;

		while (remaining >= increment) {
			// Keep track iteration when last node will be reached
			if (remaining < (NUM_THREADS * increment)) {

				// each iteration advances NUM_THREADS * increment nodes
				stop_here = remaining / increment;
			}

			for (long t = 0; t < stop_here; t++) {
				//cout << "Thread " << count << " starting at node " << node << endl;
				threads[t].start_node = node;
				threads[t].my_id = count;
				threads[t].mytsp = &tsp;
				threads[t].start();
				node += increment;
				count++;
			}

			// Wait for all the threads
			for (long t = 0; t < stop_here; t++) {
				threads[t].join();
			}
			remaining -= (stop_here * increment);
		}

		cout << "count: " << count << endl;
		// Loop through each index used and find shortest path
		for (long t = 0; t < count; t++) {
			if (tsp.path_vals[t][1] < best) {
				bestIndex = tsp.path_vals[t][0];
				best = tsp.path_vals[t][1];
			}
		}

		end = clock();
		cout << "\nbest: " << best << " @ index " << bestIndex << endl;
		cout << "time: " << ((float) (end - start)) / CPS << "s\n";

		// Store best path
		tsp.create_tour(bestIndex);
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();
		tsp.make_shorter();

		cout << "\nFinal length: " << tsp.pathLength << endl;

		// Print to file
		tsp.printResult(true);

		if (DEBUG)
			cout << "\nTotal time: " << ((float) (clock() - t)) / CPS << "s\n";	

	}

	//get result && publish
}
// void init_marker()
// {
// 	points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "map";
// 	points.ns = line_strip.ns = goal_circle.ns = "Markers";
// 	points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
// 	points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
// 	points.id = 0;
// 	line_strip.id = 1;
// 	goal_circle.id = 2;

// 	points.type = visualization_msgs::Marker::POINTS;
// 	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
// 	goal_circle.type = visualization_msgs::Marker::CYLINDER;
// 	// POINTS markers use x and y scale for width/height respectively
// 	points.scale.x = 0.1;
// 	points.scale.y = 0.1;

// 	//LINE_STRIP markers use only the x component of scale, for the line width
// 	line_strip.scale.x = 0.1;

// 	goal_circle.scale.x = 0.3;
// 	goal_circle.scale.y = 0.3;
// 	goal_circle.scale.z = 0.1;

// 	// Points are green
// 	points.color.g = 1.0f;
// 	points.color.a = 1.0;

// 	// Line strip is blue
// 	line_strip.color.b = 0.5;
// 	line_strip.color.a = 1.0;

// 	//goal_circle is yellow
// 	goal_circle.color.r = 1.0;
// 	goal_circle.color.g = 1.0;
// 	goal_circle.color.b = 0.0;
// 	goal_circle.color.a = 0.5;
// }
int main(int argc, char** argv) {
	//ros
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/center_pose_array", 1000, Center_arrayCB);
	marker_pub = n.advertise<visualization_msgs::Marker>("/Marker1", 10);
	// init_marker();
	ros::spin();

	// // Check that user entered filename on command line
	// if (argc < 2)
	// {
	// 	usage();
	// 	exit(-1);
	// }

	// // Read file names from input
	// string f, o;
	// f = o = argv[1];
	// o.append(".tour");

	// // Create new tsp object
	// TSP tsp(f, o);
	// int n = tsp.get_size();

	// // Start timing
	// clock_t t = clock();
	// clock_t t2;

	// // Read cities from file
	// if (DEBUG)
	// 	cout << "Reading cities" << endl;
	// tsp.readCities();
	// if (DEBUG)
	// 	cout << "Time to read cities: "
	// 			<< ((float) (clock() - t)) / CLOCKS_PER_SEC << " s\n";

	// cout << "number of cities: " << tsp.n << endl;

	// // Fill N x N matrix with distances between nodes
	// if (DEBUG)
	// 	cout << "\nFilling matrix" << endl;
	// t2 = clock();
	// tsp.fillMatrix_threads();
	// if (DEBUG)
	// 	cout << "Time to fill matrix: " << ((float) (clock() - t2)) / CPS
	// 			<< " s\n";

	// // Find a MST T in graph G
	// if (DEBUG)
	// 	cout << "\nFinding mst" << endl;
	// t2 = clock();
	// tsp.findMST_old();
	// if (DEBUG)
	// 	cout << "Time to find mst: " << ((float) (clock() - t2)) / CPS
	// 			<< " s\n";

	// // Find a minimum weighted matching M for odd vertices in T
	// if (DEBUG)
	// 	cout << "\nFinding perfect matching" << endl;
	// t2 = clock();
	// tsp.perfect_matching();
	// if (DEBUG)
	// 	cout << "Time to find matching: " << ((float) (clock() - t2)) / CPS
	// 			<< " s\n\n";

	// // Find the node that leads to the best path
	// clock_t start, end;
	// start = clock();

	// // Create array of thread objects
	// MyThread threads[NUM_THREADS];

	// int best = INT_MAX;
	// int bestIndex;
	// int stop_here = NUM_THREADS;

	// // Amount to increment starting node by each time
	// int increment = 1; // by 1 if n < 1040

	// if (n >= 600 && n < 1040)
	// 	increment = 3;
	// else if (n >= 1040 && n < 1800)
	// 	increment = 8;
	// else if (n >= 1800 && n < 3205)
	// 	increment = 25; 		// ~ 220s @ 3200
	// else if (n >= 3205 && n < 4005)
	// 	increment = 50; 		// ~ 230s @ 4000
	// else if (n >= 4005 && n < 5005)
	// 	increment = 120;		// ~ 200 @ 5000
	// else if (n >= 5005 && n < 6500)
	// 	increment = 250;		// ~ 220s @ 6447
	// else if (n >= 6500)
	// 	increment = 500;

	// int remaining = n;

	// // Start at node zero
	// int node = 0;

	// // Count to get thread ids
	// int count = 0;

	// while (remaining >= increment) {
	// 	// Keep track iteration when last node will be reached
	// 	if (remaining < (NUM_THREADS * increment)) {

	// 		// each iteration advances NUM_THREADS * increment nodes
	// 		stop_here = remaining / increment;
	// 	}

	// 	for (long t = 0; t < stop_here; t++) {
	// 		//cout << "Thread " << count << " starting at node " << node << endl;
	// 		threads[t].start_node = node;
	// 		threads[t].my_id = count;
	// 		threads[t].mytsp = &tsp;
	// 		threads[t].start();
	// 		node += increment;
	// 		count++;
	// 	}

	// 	// Wait for all the threads
	// 	for (long t = 0; t < stop_here; t++) {
	// 		threads[t].join();
	// 	}
	// 	remaining -= (stop_here * increment);
	// }

	// cout << "count: " << count << endl;
	// // Loop through each index used and find shortest path
	// for (long t = 0; t < count; t++) {
	// 	if (tsp.path_vals[t][1] < best) {
	// 		bestIndex = tsp.path_vals[t][0];
	// 		best = tsp.path_vals[t][1];
	// 	}
	// }

	// end = clock();
	// cout << "\nbest: " << best << " @ index " << bestIndex << endl;
	// cout << "time: " << ((float) (end - start)) / CPS << "s\n";

	// // Store best path
	// tsp.create_tour(bestIndex);
	// tsp.make_shorter();
	// tsp.make_shorter();
	// tsp.make_shorter();
	// tsp.make_shorter();
	// tsp.make_shorter();

	// cout << "\nFinal length: " << tsp.pathLength << endl;

	// // Print to file
	// tsp.printResult();

	// if (DEBUG)
	// 	cout << "\nTotal time: " << ((float) (clock() - t)) / CPS << "s\n";
	return 0;
}
