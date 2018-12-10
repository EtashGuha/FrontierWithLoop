#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "frontier_with_loop/frontier_search.h"
#include "frontier_with_loop/pose_handler.h"
#include "frontier_with_loop/visualize_points.h"
#include <visualization_msgs/Marker.h>
#include <cmath>
class Map
{
	nav_msgs::OccupancyGrid map;
	float resolution;
	int width;
	int height;
	float origin_x;
	float origin_y;
	float origin_z;
	PoseHandlers::PoseHandler pose_handler;
	std::vector<std::pair<int, int> > centroids;
	std::vector<std::pair<int, int> > example;
	std::vector<std::vector<std::pair<int, int>>>frontiers;
	VisualizePoints::VisualizePointer visualizationPointer;
	std::vector<std::pair<double, double>> centroidsTransformed;

	public:
		void initMap(){
			ros::NodeHandle n;
			ros::Subscriber sub = n.subscribe("/move_base/global_costmap/costmap", 1000, &Map::callBack, this);	
			ROS_INFO("Got here");
			ros::spin();
		}

	void callBack(nav_msgs::OccupancyGrid grid){
		resolution = grid.info.resolution;
		width = grid.info.width;
		height = grid.info.height;
		map = grid;
		FrontierSearches::FrontierSearch frontier_search(grid, pose_handler);
		frontiers = frontier_search.buildBidimensionalMap();
		centroids = frontier_search.getCentroids(frontiers);
		for(std::pair<int, int> coords: centroids){
			std::pair<double, double> newCoords;
		 	newCoords.first = (double)coords.second * grid.info.resolution + grid.info.origin.position.x;
		 	newCoords.second =  (double)coords.first * grid.info.resolution + grid.info.origin.position.y;
			centroidsTransformed.push_back(newCoords);
		}
		for(std::pair<double, double> coords: centroidsTransformed){
			ROS_INFO("first: %f, second %f", coords.first, coords.second);
		}
		visualizationPointer.visualize_lines(centroidsTransformed);
		ROS_INFO("hanwen");
		
	}
};



int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");
	Map map;
	map.initMap();
	ros::spin();
	return 0;
}

