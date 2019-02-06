#include <ros/ros.h>
#include <frontier_with_loop/visualize_points.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

namespace VisualizePoints{

	VisualizePointer::VisualizePointer(){
		ros::NodeHandle n;
		marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	}

	void VisualizePointer::visualize_lines(std::vector<std::pair<double, double>> pointsToHighlight){
			ros::Rate r(30);

			float f = 0.0;
			if(ros::ok()){
		    visualization_msgs::Marker points;
		    points.header.frame_id = "/map";
		    points.header.stamp = ros::Time::now();
		    points.ns = "points_and_lines";
		    points.action = visualization_msgs::Marker::ADD;
		    points.pose.orientation.w = 1.0;

		    points.id = 0;

		    points.type = visualization_msgs::Marker::POINTS;

		    // POINTS markers use x and y scale for width/height respectively
		    points.scale.x = 0.2;
		    points.scale.y = 0.2;

		    // Points are green
		    points.color.g = 1.0f;
		    points.color.a = 1.0;

		    for(std::pair<double, double> point : pointsToHighlight){
				geometry_msgs::Point p;
				p.x = ((double)point.first);
				p.y = ((double)point.second);
				p.z = 0;

		     	points.points.push_back(p);
			}
		    marker_pub.publish(points);

		    f += 0.04;
		    r.sleep();
		}

}}


