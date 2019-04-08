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
		bool isFrontier;
	};

	typedef std::pair<int, int> coordinate;

	FastMarch::FastMarch(){}
	std::pair<coordinate, coordinate> FastMarch::march(nav_msgs::OccupancyGrid map, std::vector<std::pair<int,int>> frontierPoints){
		printf("marching\n");
		std::queue<point> q; 
		std::unordered_map<coordinate, int, boost::hash<coordinate>> isLabeled;
		std::unordered_map<int, coordinate> labelToFrontierPoint;
		int label = 0;
		for (auto & frontierPoint: frontierPoints) {
			point p;
			p.x = frontierPoint.first;
	    	p.y = frontierPoint.second;
	    	p.label = label;
	    	p.isFrontier = true;
			printf("FIRST PAIR : (%0.2f, %0.2f)\n",
				(double)p.x * map.info.resolution + map.info.origin.position.x, 
				(double)p.y * map.info.resolution + map.info.origin.position.y);
	    	q.push(p);
			labelToFrontierPoint[label] = {frontierPoint.first, frontierPoint.second};
	    	isLabeled[{frontierPoint.first, frontierPoint.second}] = label++;
		}
		printf("beginning while loop \n");
		for(auto it = labelToFrontierPoint.cbegin(); it != labelToFrontierPoint.cend(); ++it){
			printf("label: %d  frontier x: %0.2f  frontier y: %0.2f\n",
				it -> first, 
				(double)it -> second.first * map.info.resolution + map.info.origin.position.x,
				(double)it -> second.second * map.info.resolution + map.info.origin.position.y);
		}	

		// for(auto it = isLabeled.cbegin(); it != isLabeled.cend(); ++it)
		// {
		// 	printf("label: %d  frontier x: %0.2f  frontier y: %0.2f\n",
		// 		it -> second, 
		// 		(double)it -> first.first * map.info.resolution + map.info.origin.position.x,
		// 		(double)it -> first.second * map.info.resolution + map.info.origin.position.y);
		// }	
		std::queue<point> copyQ (q);
		printf("queue Values\n");
		//printing content of queue 
		while (!copyQ.empty()){
			printf("X: %0.2f  Y: %0.2f \n", 
				(double)copyQ.front().x * map.info.resolution + map.info.origin.position.x,
				(double)copyQ.front().y * map.info.resolution + map.info.origin.position.y);
			copyQ.pop();
		}
		printf("end of queue\n");
		VisualizePoints::VisualizePointer visualizationPointer;
		std::vector<std::pair<double, double>> pointsToHighlight;
		while(q.size() != 0){
			point p = q.front();
			q.pop();
			std::queue<point> copyQ (q);
			// for(auto it = isLabeled.cbegin(); it != isLabeled.cend(); ++it)
			// {
			// 	printf("label: %d  frontier x: %0.2f  frontier y: %0.2f\n",
			// 		it -> second, 
			// 		(double)it -> first.first * map.info.resolution + map.info.origin.position.x,
			// 		(double)it -> first.second * map.info.resolution + map.info.origin.position.y);
			// }
			// printf("COUNT: %d", isLabeled.count({1000, 1000}));
			// printf("queue Values\n");
			// //printing content of queue 
			// while (!copyQ.empty()){
			// 	printf("hello\n");
			// 	//printf("X: %0.2f  Y: %0.2f \n", 
			// 	//	(double)copyQ.front().x * map.info.resolution + map.info.origin.position.x,
			// 	//	(double)copyQ.front().y * map.info.resolution + map.info.origin.position.y);
			// 	copyQ.pop();
			// }
			//printf("end of queue\n");

			if(p.x >= map.info.width || p.x < 0 || p.y >= map.info.height || p.y < 0){
				printf("out of bounds x: %0.2f  y %0.2f\n",
					(double)p.x * map.info.resolution + map.info.origin.position.x,
					(double)p.y * map.info.resolution + map.info.origin.position.y);
				continue;
			}

			if(map.data[p.x * map.info.height + p.y] > 0){
				printf("Inside the map x: %0.2f  y %0.2f\n",
					(double)p.x * map.info.resolution + map.info.origin.position.x,
					(double)p.y * map.info.resolution + map.info.origin.position.y);
				continue;
			}

			if(isLabeled.count({p.x, p.y}) == 0){
				printf("addign new pint\n");
				isLabeled[{p.x, p.y}] = p.label;
			} else if(isLabeled[{p.x, p.y}] != p.label){
				printf("Label of Point: %d\n", p.label);
				coordinate curr = labelToFrontierPoint.at(p.label);
				for(auto it = labelToFrontierPoint.cbegin(); it != labelToFrontierPoint.cend(); ++it){
					printf("label: %d  frontier x: %0.2f  frontier y: %0.2f\n",
						it -> first, 
						(double)it -> second.first * map.info.resolution + map.info.origin.position.x,
						(double)it -> second.second * map.info.resolution + map.info.origin.position.y);
				}
				
				coordinate next = labelToFrontierPoint.at(isLabeled.at({p.x, p.y}));
				std::pair<coordinate, coordinate> ans;
				ans.first = curr;
				ans.second = next;
				printf("Returning FIRST PAIR : (%0.2f, %0.2f)   SECOND PAIR: (%0.2f, %0.2f)\n", 
					(double)curr.first * map.info.resolution + map.info.origin.position.x,
				 	(double)curr.second  * map.info.resolution + map.info.origin.position.y, 
				 	(double)next.first  * map.info.resolution + map.info.origin.position.x, 
				 	(double)next.second * map.info.resolution + map.info.origin.position.y);
				return ans;
			} else if(!p.isFrontier){
				printf("continuing \n");
				continue;
			}
			printf("Current: (%0.2f, %0.2f)\n", (double)p.x * map.info.resolution + map.info.origin.position.x, (double)p.y * map.info.resolution + map.info.origin.position.y);
			point nextPoint;
			nextPoint.label = p.label;
			nextPoint.x = p.x + 1;
			nextPoint.y = p.y;
			nextPoint.isFrontier = false;
			q.push(nextPoint);
			point nextPointTwo;
			nextPointTwo.label = p.label;
			nextPointTwo.x = p.x - 1;
			nextPointTwo.y = p.y;
			nextPoint.isFrontier = false;
			q.push(nextPointTwo);
			point nextPointThree;
			nextPointThree.label = p.label;
			nextPointThree.x = p.x;
			nextPointThree.y = p.y + 1;
			nextPoint.isFrontier = false;
			q.push(nextPointThree);
			point nextPointFour;
			nextPointFour.label = p.label;
			nextPointFour.x = p.x;
			nextPointFour.y = p.y - 1;
			nextPoint.isFrontier = false;
			q.push(nextPointFour);
		}
	}
}

