#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
class Map
{
	nav_msgs::OccupancyGrid map;
	float resolution;
	int width;
	int height;
	float origin_x;
	float origin_y;
	float origin_z;
	public:
		void initMap(){
			ros::NodeHandle n;
			ros::Subscriber sub = n.subscribe("/move_base/global_costmap/costmap", 1000, &Map::callBack, this);
			ros::spin();
			ROS_INFO("Got here");
		}

	void callBack(nav_msgs::OccupancyGrid grid){
		resolution = grid.info.resolution;
		width = grid.info.width;
		height = grid.info.height;
		map = grid;
	}
};



int main(int argc, char **argv){
	ros::init(argc, argv, "MapPeeker");
	Map map;
	map.initMap();
	ros::spin();
	return 0;
}

