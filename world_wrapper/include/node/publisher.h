#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "world_wrapper/world_wrapper.h"

namespace ww{


class Publisher{

	public:

    Publisher(std::string name,ros::NodeHandle *n,ww::World_wrapper* world_wrapper);

    void init(const std::string& frame_id);

    void init_test_point(const std::string& frame_id);

    void init_feature_marker(const std::string& frame_id);

    void init_corner_marker(const std::string& frame_id);

    void init_surface_marker(const std::string& frame_id);

    void init_edge_marker(const std::string& frame_id);

    void update_position_test_point(const geo::fCVec3& P);

    void update_position();

    void publish();

private:

    void update_position_surface();
    void update_position_edge();
    void update_position_corner();
    void update_position_features();



private:

    std::string name;
    ros::NodeHandle* ptr_n;
    ww::World_wrapper* ptr_world_wrapper;

    geo::fCVec3 p_surface;
    geo::fCVec3 p_edge;
    geo::fCVec3 p_corner;


    ros::Publisher wrap_pub,surface_pub,test_point_pub,feature_pub,edge_pub;
    visualization_msgs::Marker  marker_test_point;
    visualization_msgs::Marker marker_corners;

    visualization_msgs::MarkerArray marker_features;
    visualization_msgs::MarkerArray marker_surfaces_array;
    visualization_msgs::MarkerArray edge_marker_array;



};

}

#endif
