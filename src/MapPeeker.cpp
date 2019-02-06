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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Map
{
	float resolution;
	int width;
	int height;
	bool what;
	float origin_x;
	float origin_y;
	float origin_z;
	float current_x;
	float current_y;
	bool always_send_full_costmap;
	PoseHandlers::PoseHandler pose_handler;
	std::vector<std::pair<int, int> > centroids;
	std::vector<std::pair<int, int> > example;
	std::vector<std::vector<std::pair<int, int>>>frontiers;
	VisualizePoints::VisualizePointer visualizationPointer;
	std::vector<std::pair<double, double>> centroidsTransformed;
	ros::NodeHandle n;
	public:
		nav_msgs::OccupancyGrid map;
		ros::Subscriber subFirst;
		void initMap(){	
			ROS_INFO("Starting first subcriber");
			ros::Subscriber poseUpdater = n.subscribe("/pose", 1, &Map::updateCurrPose, this);
			ros::spin();
		}
		void updateFrontiers(){
			MoveBaseClient ac("move_base", true);
			centroidsTransformed.clear();
			resolution = map.info.resolution;
			width = map.info.width;
			height = map.info.height;
			FrontierSearches::FrontierSearch frontier_search(map, pose_handler);
			frontiers = frontier_search.buildBidimensionalMap();
			centroids = frontier_search.getCentroids(frontiers);
			for(std::pair<int, int> coords: centroids){
				std::pair<double, double> newCoords;
		 		newCoords.first = (double)coords.second * map.info.resolution + map.info.origin.position.x;
		 		newCoords.second =  (double)coords.first * map.info.resolution + map.info.origin.position.y;
				centroidsTransformed.push_back(newCoords);
				//printf("X: %f, Y: %f\n", newCoords.first, newCoords.second );
			}
			visualizationPointer.visualize_lines(centroidsTransformed);

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "base_link";
  			goal.target_pose.header.stamp = ros::Time::now();
  			printf("first: %f, second, %f\n", (*centroidsTransformed.begin()).first, (*centroidsTransformed.begin()).second);
  			goal.target_pose.pose.position.x = -1.0 * ((*centroidsTransformed.begin()).first - current_x);
  			goal.target_pose.pose.position.y = -1.0* ((*centroidsTransformed.begin()).second - current_y);
  			goal.target_pose.pose.orientation.w = 1.0;
  			printf("Robotx: %f, Roboty, %f\n", current_x, current_y);
  			printf("Goalx: %f, Goaly %f\n", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  			ROS_INFO("SENDING GOAL");
  			ac.sendGoal(goal);
  			ac.waitForResult();

  			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
   			 	ROS_INFO("Hooray, the base moved 1 meter forward");
   			 	updateFrontiers();
   			 }
  			else{
    			ROS_INFO("The base failed to move forward 1 meter for some reason");
  			}
		}


	void callBack(nav_msgs::OccupancyGrid grid){
		map = grid;
		ROS_INFO("Calling update frontiers");
		updateFrontiers();
	}

	void updateCurrPose(geometry_msgs::PoseStamped currentPosition){
		ROS_INFO("Running firt subcriber");
		printf("Currentx: %f, Currenty: %f\n", current_x, current_y);
		current_x = currentPosition.pose.position.x;
		current_y = currentPosition.pose.position.y;
		subFirst = n.subscribe("/move_base/global_costmap/costmap", 1, &Map::callBack, this);	
	}





};



int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");

	Map map;
	map.initMap();
	return 0;
}

