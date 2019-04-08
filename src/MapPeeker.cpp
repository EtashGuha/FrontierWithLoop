#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "frontier_with_loop/frontier_search.h"
#include "frontier_with_loop/pose_handler.h"
#include "frontier_with_loop/visualize_points.h"
#include "frontier_with_loop/fast_march.h"
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
		PoseHandlers::PoseHandler pose_handler;
		std::vector<std::pair<int, int> > centroids;
		std::vector<std::pair<int, int> > centroidsReversed;
		std::vector<std::vector<std::pair<int, int>>>frontiers;
		VisualizePoints::VisualizePointer visualizationPointer;
		std::vector<std::pair<double, double>> centroidsTransformed;
		std::shared_ptr<message_filters::Subscriber<nav_msgs::OccupancyGrid> > sub;
		std::shared_ptr<message_filters::Cache<nav_msgs::OccupancyGrid> > cache;
		ros::NodeHandle n;
		MoveBaseClient ac;
		ros::Timer timer;
void cb(nav_msgs::OccupancyGrid map);

		Map();

		void initMap();
		bool updateFrontiers(nav_msgs::OccupancyGrid map);
		void timerCallback(const ros::TimerEvent& e);

};
		
Map::Map(): n(), ac(n, "move_base", true) {}//, sub(n, "/move_base/global_costmap/costmap", 1), cache(sub, 100){}

void Map::initMap(){
	ROS_INFO("Starting first subcriber");
	n.setParam("/move_base/global_costmap/always_send_full_costmap", true);
	n.setParam("/move_base/global_costmap/costmap/always_send_full_costmap", true);
	sub = std::make_shared<message_filters::Subscriber<nav_msgs::OccupancyGrid> >(n, "/move_base/global_costmap/costmap", 10);
	cache = std::make_shared<message_filters::Cache<nav_msgs::OccupancyGrid> >(*sub,1);

	sub->registerCallback(&Map::cb,this);

	Map::timer = n.createTimer(ros::Duration(0.01), &Map::timerCallback, this, true);
	//Map::timerCallback();
}

void Map::cb(nav_msgs::OccupancyGrid map){
	//ROS_INFO_STREAM("Got message with time " << map.header.stamp);
}

void Map::timerCallback(const ros::TimerEvent& e) {
	bool keep_going=true;
	ros::Rate r(10);
	while(ros::ok() && keep_going) {
		ros::Time t = Map::cache->getLatestTime();
		ROS_INFO_STREAM("Latest time: " << t);
		std::vector<nav_msgs::OccupancyGrid::ConstPtr> v = Map::cache->getInterval(t,t);
		if(v.size() > 0){
			keep_going = Map::updateFrontiers(*v[0]);
		}
		r.sleep();
	}
}

bool Map::updateFrontiers(nav_msgs::OccupancyGrid map){
	//ROS_INFO("updating frontiers");
	centroidsTransformed.clear();
	centroids.clear();
	FrontierSearches::FrontierSearch frontier_search(map, pose_handler);
	frontiers = frontier_search.buildBidimensionalMap();
	centroidsReversed = frontier_search.getCentroids(frontiers);
	for(std::pair<int, int> coords: centroidsReversed){
		std::pair<int, int> currCoordinate;
		currCoordinate.first = coords.second;
		currCoordinate.second = coords.first;
		centroids.push_back(currCoordinate);
	}
	for(std::pair<int, int> coords: centroids){
		std::pair<double, double> newCoords;
 		newCoords.first = (double)coords.first * map.info.resolution + map.info.origin.position.x;
 		newCoords.second =  (double)coords.second * map.info.resolution + map.info.origin.position.y;
		centroidsTransformed.push_back(newCoords);
	}
	visualizationPointer.visualize_lines(centroidsTransformed);
	FastMarch::FastMarch fastMarch;
	std::pair< std::pair<int, int>, std::pair<int, int>> closestFrontiers = fastMarch.march(map, centroids);
	// printf("FIRST PAIR: (%0.2f, %0.2f) SECOND PAIR: (%0.2f, %0.2f)\n",
	// 	(double)closestFrontiers.first.first * map.info.resolution + map.info.origin.position.x,
	// 	(double)closestFrontiers.first.second * map.info.resolution + map.info.origin.position.y,
	// 	(double)closestFrontiers.second.first * map.info.resolution + map.info.origin.position.x,
	// 	(double)closestFrontiers.second.second * map.info.resolution + map.info.origin.position.y);
		
	printf("done with fast march\n");
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.pose.position.x = ((double)closestFrontiers.first.first * map.info.resolution + map.info.origin.position.x);
	goal.target_pose.pose.position.y = ((double)closestFrontiers.first.second * map.info.resolution + map.info.origin.position.y);
	goal.target_pose.pose.orientation.w = 1.0;
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	 	ROS_INFO("Got to goal");
	 	return true;
	 }
	else{
		ROS_INFO("Did not get to goal");
		return false;
	}
}	


int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");

	Map map;
	map.initMap();
	//ros::spin();
	ros::MultiThreadedSpinner spin(4);
	spin.spin();

	return 0;
}

