#ifndef FAST_MARCH_H
#define FAST_MARCH_H

namespace FastMarch{
	class FastMarch{
	public:
		FastMarch();
		std::pair<std::pair<int,int>,std::pair<int,int>> march(nav_msgs::OccupancyGrid map, std::vector<std::pair<int,int>> frontierPoints);
	};
}
#endif