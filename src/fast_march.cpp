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
#include <frontier_with_loop/fast_march.h>

namespace FastMarch{
	struct point{
		int x;
		int y;
		int label;
		int distance;
	};

	typedef std::pair<int, int> coordinate;

	FastMarch::FastMarch(){}

	std::pair<coordinate, coordinate> FastMarch::march(nav_msgs::OccupancyGrid map, std::vector<std::pair<int,int>> frontierPoints){
		std::queue<point> q; 
		std::unordered_map<coordinate, int, boost::hash<coordinate>> isLabeled;
		std::unordered_map<coordinate, int, boost::hash<coordinate>> pointToDistance;
		std::unordered_map<int, coordinate> labelToFrontierPoint;
		int label = 0;
		for (auto & frontierPoint: frontierPoints) {
			point p;
			p.x = frontierPoint.first;
	    	p.y = frontierPoint.second;

			//printf("x: %d  y: %d", p.x, p.y);
	    	p.label = label;
	    	p.distance = 0;
	    	q.push(p);
			labelToFrontierPoint[label] = {frontierPoint.first, frontierPoint.second};
	    	isLabeled[{frontierPoint.first, frontierPoint.second}] = label++;
	    	pointToDistance[{frontierPoint.first, frontierPoint.second}] = 0;
		}
		printf("queue size %d \n", q.size());
		while(q.size() != 0){
			point p = q.front();
			q.pop();
			if(p.x >= map.info.width || p.x < 0 || p.y >= map.info.height || p.y < 0){
				continue;
			}
			//printf("x: %d  y: %d\n", p.x, p.y);
			if(map.data[p.y * map.info.width + p.x] > 0){
				continue;
			}
			if(isLabeled.count({p.x, p.y}) == 0){
				isLabeled[{p.x, p.y}] = p.label;
				pointToDistance[{p.x, p.y}] = p.distance;
				continue;
			} else if(isLabeled[{p.x, p.y}] != p.label){
				coordinate curr = labelToFrontierPoint[p.label];
				coordinate next = labelToFrontierPoint[isLabeled[{p.x, p.y}]];
				std::pair<coordinate, coordinate> ans;
				ans.first = curr;
				ans.second = next;
				return ans;
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
}

