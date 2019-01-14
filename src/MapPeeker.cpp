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
	bool what;
	float origin_x;
	float origin_y;
	float origin_z;
	bool always_send_full_costmap;
	PoseHandlers::PoseHandler pose_handler;
	std::vector<std::pair<int, int> > centroids;
	std::vector<std::pair<int, int> > example;
	std::vector<std::vector<std::pair<int, int>>>frontiers;
	VisualizePoints::VisualizePointer visualizationPointer;
	std::vector<std::pair<double, double>> centroidsTransformed;

	public:
		void initMap(){			
			ros::NodeHandle n;
			ros::Subscriber subFirst = n.subscribe("/move_base/global_costmap/costmap", 1, &Map::callBack, this);	
			ros::spin();
		}

	void callBack(nav_msgs::OccupancyGrid grid){
		centroidsTransformed.clear();
		resolution = grid.info.resolution;
		width = grid.info.width;
		height = grid.info.height;
		FrontierSearches::FrontierSearch frontier_search(grid, pose_handler);
		frontiers = frontier_search.buildBidimensionalMap();
		centroids = frontier_search.getCentroids(frontiers);
		for(std::pair<int, int> coords: centroids){
			std::pair<double, double> newCoords;
		 	newCoords.first = (double)coords.second * grid.info.resolution + grid.info.origin.position.x;
		 	newCoords.second =  (double)coords.first * grid.info.resolution + grid.info.origin.position.y;
			centroidsTransformed.push_back(newCoords);
		}
		visualizationPointer.visualize_lines(centroidsTransformed);
	}
};



int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");
	Map map;
	map.initMap();
	return 0;
}

