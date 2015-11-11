#include "visualisation/vis_wbox.h"

namespace ww_vis{


Vis_wbox::Vis_wbox(ros::NodeHandle& node,float scale):
scale(scale)
{
    corner_pub     = node.advertise<visualization_msgs::MarkerArray>("wbox_corner", 10);
    surface_pub    = node.advertise<visualization_msgs::MarkerArray>("wbox_surface",10);
    edge_pub       = node.advertise<visualization_msgs::MarkerArray>("wbox_edge",10);
}

void Vis_wbox::push_back(const wobj::WBox* ptr_wbox){
    wboxes.push_back(ptr_wbox);
}

void Vis_wbox::initialise(){

    edges_ma.markers.clear();
    surfaces_ma.markers.clear();
    corners_ma.markers.clear();

    init_corner_marker();
    init_edge_marker();
    init_surface_marker();

    update();
}

void Vis_wbox::init_corner_marker(){

    for(std::size_t i = 0; i < wboxes.size() * 8;i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/world";
            marker.type            = visualization_msgs::Marker::SPHERE_LIST;
            marker.scale.x         = scale;
            marker.scale.y         = scale;
            marker.scale.z         = scale;
            marker.color.g         = 1.0f;
            marker.color.a         = 1.0f;
            marker.points.resize(1);
            corners_ma.markers.push_back(marker);
    }
}

void Vis_wbox::init_edge_marker(){

    for(std::size_t i = 0; i < wboxes.size() * 12;i++){
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/world";
        marker.type            = visualization_msgs::Marker::LINE_LIST;
        marker.color.a         = 1.0f;
        marker.color.b         = 1.0f;
        marker.scale.x         = scale;
        marker.scale.y         = scale;
        marker.scale.z         = scale;
        marker.points.resize(2);
        edges_ma.markers.push_back(marker);
    }

}

void Vis_wbox::init_surface_marker(){

    for(std::size_t i = 0; i < wboxes.size() * 6;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.color.a         = 1.0f;
        marker.color.r         = 1.0f;
        marker.scale.x         = 0.01;
        marker.scale.y         = 0.01;
        marker.scale.z         = 0.01;
        marker.points.resize(2);
        surfaces_ma.markers.push_back(marker);
    }
}

void Vis_wbox::update_position_edge(){
    std::size_t index = 0;
    for(std::size_t i = 0; i < wboxes.size();i++){
        const wobj::WBox& wbox_i =  (*wboxes[i]);
        for(std::size_t k = 0; k < 12; k++){
            edges_ma.markers[index].header.stamp = ros::Time::now();
            edges_ma.markers[index].id = index;
            edges_ma.markers[index].points[0].x = wbox_i.edges[k].P1(0);
            edges_ma.markers[index].points[0].y = wbox_i.edges[k].P1(1);
            edges_ma.markers[index].points[0].z = wbox_i.edges[k].P1(2);

            edges_ma.markers[index].points[1].x = wbox_i.edges[k].P2(0);
            edges_ma.markers[index].points[1].y = wbox_i.edges[k].P2(1);
            edges_ma.markers[index].points[1].z = wbox_i.edges[k].P2(2);
            index++;
        }
    }

}

void Vis_wbox::update_position_surface(){

    std::size_t index = 0;
    geo::fCVec3 mid,n;

    for(std::size_t i = 0; i < wboxes.size();i++){
        const wobj::WBox& wbox_i =  (*wboxes[i]);
        for(std::size_t k = 0; k < 6;k++){

            mid = wbox_i.surfaces[k].get_middle();
            n   = wbox_i.surfaces[k].get_normal();

            surfaces_ma.markers[index].header.stamp = ros::Time::now();
            surfaces_ma.markers[index].id = index;
            surfaces_ma.markers[index].points[0].x = mid(0);
            surfaces_ma.markers[index].points[0].y = mid(1);
            surfaces_ma.markers[index].points[0].z = mid(2);

            surfaces_ma.markers[index].points[1].x = 0.1*n(0) + mid(0);
            surfaces_ma.markers[index].points[1].y = 0.1*n(1) + mid(1);
            surfaces_ma.markers[index].points[1].z = 0.1*n(2) + mid(2);
            index++;
        }
    }


}

void Vis_wbox::update_position_corner(){
    std::size_t index = 0;
    assert(corners_ma.markers.size() == wboxes.size() * 8);
    for(std::size_t i = 0; i < wboxes.size();i++){
        const wobj::WBox& wbox_i =  (*wboxes[i]);
        for(std::size_t k = 0; k < 8;k++){

            corners_ma.markers[index].header.stamp = ros::Time::now();
            corners_ma.markers[index].id = index;
            corners_ma.markers[index].points[0].x = wbox_i.corners[k].C(0);
            corners_ma.markers[index].points[0].y = wbox_i.corners[k].C(1);
            corners_ma.markers[index].points[0].z = wbox_i.corners[k].C(2);
            index++;
        }
    }


}

void Vis_wbox::update(){
    update_position_corner();
    update_position_edge();
    update_position_surface();
}

void Vis_wbox::publish(){
    corner_pub.publish(corners_ma);
    edge_pub.publish(edges_ma);
    surface_pub.publish(surfaces_ma);
}

}
