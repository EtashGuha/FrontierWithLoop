#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "frontier_with_loop/frontier_search.h"
#include "frontier_with_loop/pose_handler.h"
#include "frontier_with_loop/visualize_points.h"
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <boost/functional/hash.hpp>
#include <queue>  
#include <unordered_map> 
struct point{
	int x;
	int y;
	int label;
	int distance;
};
typedef std::pair<int, int> coordinate;
int march(nav_msgs::OccupancyGrid map, point first, point second){
	std::queue<point> q; 
	std::unordered_map<coordinate, int, boost::hash<coordinate>> isLabeled;
	std::unordered_map<coordinate, int, boost::hash<coordinate>> pointToDistance;

	isLabeled[{first.x, first.y}] = first.label;
	isLabeled[{second.x, second.y}] = second.label;
	pointToDistance[{first.x, first.y}] = first.distance;
	pointToDistance[{second.x, second.y}] = second.distance;
	q.push(first);
	q.push(second);

	while(q.size() != 0){
		point p = q.front();
		q.pop();
		if(isLabeled.count({p.x, p.y}) == 0){
			isLabeled[{p.x, p.y}] = p.label;
			pointToDistance[{p.x, p.y}] = p.distance;
		} else if(isLabeled[{p.x, p.y}] != p.label){
			return 2 * p.distance;
		} else if(p.distance < pointToDistance[{p.x, p.y}]) {
			pointToDistance[{p.x, p.y}] = p.distance;
		}
		point nextPoint;
		nextPoint.label = p.label;
		nextPoint.distance = p.distance + 1;
		nextPoint.x = p.x + 1;
		nextPoint.y = p.y;
		q.push(nextPoint);
		point nextPointTwo;
		nextPointTwo.label = p.label;
		nextPointTwo.distance = p.distance + 1;
		nextPointTwo.x = p.x - 1;
		nextPointTwo.y = p.y;
		q.push(nextPointTwo);
		point nextPointThree;
		nextPointThree.label = p.label;
		nextPointThree.distance = p.distance + 1;
		nextPointThree.x = p.x;
		nextPointThree.y = p.y + 1;
		q.push(nextPointThree);
		point nextPointFour;
		nextPointFour.label = p.label;
		nextPointFour.distance = p.distance + 1;
		nextPointFour.x = p.x;
		nextPointFour.y = p.y - 1;
		q.push(nextPointFour);
	}
}
int main(int argc, char **argv){
	ros::init(argc, argv, "fast_march");
	return 0;
}

