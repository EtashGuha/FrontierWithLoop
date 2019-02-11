#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "frontier_with_loop/frontier_search.h"
#include "frontier_with_loop/pose_handler.h"
#include "frontier_with_loop/visualize_points.h"
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/cache.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Map {
	public:
		bool always_send_full_costmap;
		PoseHandlers::PoseHandler pose_handler;
		std::vector<std::pair<int, int> > centroids;
		std::vector<std::vector<std::pair<int, int>>>frontiers;
		VisualizePoints::VisualizePointer visualizationPointer;
		std::vector<std::pair<double, double>> centroidsTransformed;
		ros::Subscriber subFirst;
		ros::Publisher goal_setter;
		nav_msgs::OccupancyGrid map;
		ros::NodeHandle n;

		void initMap();
		void callBack(const nav_msgs::OccupancyGrid& grid);
		void updateFrontiers();

		Map(): map(){}
};
		void Map::initMap(){
			ROS_INFO("Starting first subcriber");
			//subFirst = n.subscribe("/move_base/global_costmap/costmap", 1, &Map::callBack, this);
			message_filters::Subscriber<nav_msgs::OccupancyGrid> sub(Map::n, "/move_base/global_costmap/costmap", 1);
			message_filters::Cache<nav_msgs::OccupancyGrid> cache(sub, 100);
			sub.registerCallback(Map::callBack);
		}

		void Map::callBack(const nav_msgs::OccupancyGrid& grid){
			map = grid;
		}

		void Map::updateFrontiers(){
			ROS_INFO("updating frontiers");
			centroidsTransformed.clear();
			MoveBaseClient ac("move_base", true);
			FrontierSearches::FrontierSearch frontier_search(map, pose_handler);
			frontiers = frontier_search.buildBidimensionalMap();
			centroids = frontier_search.getCentroids(frontiers);
			for(std::pair<int, int> coords: centroids){
				std::pair<double, double> newCoords;
		 		newCoords.first = (double)coords.second * map.info.resolution + map.info.origin.position.x;
		 		newCoords.second =  (double)coords.first * map.info.resolution + map.info.origin.position.y;
				centroidsTransformed.push_back(newCoords);
			}
			visualizationPointer.visualize_lines(centroidsTransformed);
			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "/map";
			goal.target_pose.pose.position.x = ((*centroidsTransformed.begin()).first);
  			goal.target_pose.pose.position.y = ((*centroidsTransformed.begin()).second);
  			goal.target_pose.pose.orientation.w = 1.0;
  			ac.sendGoal(goal);
  			ac.waitForResult();
  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
   			 	ROS_INFO("Got to goal");
   			 }
  			else{
    			ROS_INFO("Did not get to goal");
  			}
		}	


int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");

	Map map;
	map.initMap();
	return 0;
}

