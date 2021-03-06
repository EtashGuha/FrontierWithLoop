#ifndef VISUALIZE_POINTS_H
#define VISUALIZE_POINTS_H

#include <visualization_msgs/Marker.h>
#include <cmath>

namespace VisualizePoints{
	class VisualizePointer
	{
	public:
		VisualizePointer();
		ros::Publisher marker_pub;
		void visualize_lines(std::vector<std::pair<double, double>> points);
	};

}

#endif 