#ifndef VIS_SOCKET_H_
#define VIS_SOCKET_H_


#include "wsocket.h"

// ROS

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <tf/LinearMath/Matrix3x3.h>

namespace obj{

class Vis_socket{

public:

    Vis_socket(ros::NodeHandle& n, const wobj::WSocket &wsocket);

    void initialise(std::size_t num_points, const std::string& world_frame,float scale);

    void publish();

private:

    void initialise_disk(std::vector<geometry_msgs::Point>& points, const geo::fCVec3& C, const geo::fMat33& R, const float r);

private:

    const wobj::WSocket&                wsocket;
    ros::Publisher                      socket_pub;
    visualization_msgs::MarkerArray     socket_marker_array;

};

}

#endif
