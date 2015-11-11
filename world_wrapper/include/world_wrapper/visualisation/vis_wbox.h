#ifndef VIS_WBOX_H_
#define VIS_WBOX_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "wbox.h"


namespace ww_vis{

class Vis_wbox{

public:

    Vis_wbox(ros::NodeHandle& node, float scale);

    void push_back(const wobj::WBox* ptr_wbox);

    void initialise();

    void update();

    void publish();

private:

    void init_corner_marker();

    void init_edge_marker();

    void init_surface_marker();

    void update_position_edge();

    void update_position_surface();

    void update_position_corner();

private:

    ros::Publisher surface_pub,edge_pub,corner_pub;

    std::vector<const wobj::WBox*>   wboxes;

    visualization_msgs::MarkerArray corners_ma;
    visualization_msgs::MarkerArray surfaces_ma;
    visualization_msgs::MarkerArray edges_ma;

    float                           scale;



};

}

#endif
