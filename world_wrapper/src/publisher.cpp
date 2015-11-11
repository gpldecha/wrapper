#include "node/publisher.h"

namespace ww{

Publisher::Publisher(std::string name, ros::NodeHandle *n,ww::World_wrapper* world_wrapper):
   name(name),ptr_n(n),ptr_world_wrapper(world_wrapper)
{
     wrap_pub       = ptr_n->advertise<visualization_msgs::Marker>(name + "_corner", 10);
     surface_pub    = ptr_n->advertise<visualization_msgs::MarkerArray>(name + "_surface",10);
     test_point_pub = ptr_n->advertise<visualization_msgs::Marker>(name + "_test_point",10);
     feature_pub    = ptr_n->advertise<visualization_msgs::MarkerArray>(name + "_features",10);
     edge_pub       = ptr_n->advertise<visualization_msgs::MarkerArray>(name + "_edge",10);
}

void Publisher::init(const std::string& frame_id){
    init_corner_marker(frame_id);
    init_surface_marker(frame_id);
    init_edge_marker(frame_id);
    init_feature_marker(frame_id);
    init_test_point(frame_id);
}

void Publisher::init_edge_marker(const std::string& frame_id){


    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();
    edge_marker_array.markers.resize(12 * nbObjects);

    for(std::size_t i = 0; i < edge_marker_array.markers.size();i++){
        edge_marker_array.markers[i].header.frame_id = frame_id;
        edge_marker_array.markers[i].type            = visualization_msgs::Marker::LINE_LIST;
        edge_marker_array.markers[i].color.a         = 1.0f;
        edge_marker_array.markers[i].color.b         = 1.0f;
        edge_marker_array.markers[i].scale.x         = 0.001;
        edge_marker_array.markers[i].scale.y         = 0.001;
        edge_marker_array.markers[i].scale.z         = 0.001;
        edge_marker_array.markers[i].points.resize(2);
    }

}

void Publisher::init_corner_marker(const std::string& frame_id){

    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();
    marker_corners.points.resize(8 * nbObjects);
    marker_corners.header.frame_id = frame_id;
    marker_corners.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_corners.scale.x = 0.01;
    marker_corners.scale.y = 0.01;
    marker_corners.scale.z = 0.01;
    marker_corners.color.g = 1.0f;
    marker_corners.color.a = 1.0f;
}

void Publisher::init_test_point(const std::string& frame_id){

    marker_test_point.points.resize(1);
    marker_test_point.header.frame_id = frame_id;
    marker_test_point.type            = visualization_msgs::Marker::SPHERE_LIST;
    marker_test_point.scale.x = 0.01;
    marker_test_point.scale.y = 0.01;
    marker_test_point.scale.z = 0.01;
    marker_test_point.color.r = 1.0f;
    marker_test_point.color.g = 1.0f;
    marker_test_point.color.b = 1.0f;
    marker_test_point.color.a = 1.0f;

}

void Publisher::init_feature_marker(const std::string& frame_id){

    p_surface   = ptr_world_wrapper->wrapped_objects.get_closest_point_surface();
    p_edge      = ptr_world_wrapper->wrapped_objects.get_closest_point_edge();
    p_corner    = ptr_world_wrapper->wrapped_objects.get_closest_point_surface();

    marker_features.markers.resize(3);
    for(std::size_t i = 0; i < 3;i++){

        marker_features.markers[i].header.frame_id = frame_id;
        marker_features.markers[i].type            = visualization_msgs::Marker::SPHERE_LIST;
        marker_features.markers[i].color.a         = 1.0f;
        marker_features.markers[i].scale.x         = 0.01;
        marker_features.markers[i].scale.y         = 0.01;
        marker_features.markers[i].scale.z         = 0.01;
        marker_features.markers[i].points.resize(1);
        marker_features.markers[i].id              = i;

    }

    marker_features.markers[0].points[0].x = p_surface(0);
    marker_features.markers[0].points[0].y = p_surface(1);
    marker_features.markers[0].points[0].z = p_surface(2);
    marker_features.markers[0].color.r     = 1;
    marker_features.markers[0].color.g     = 0;
    marker_features.markers[0].color.b     = 0;



    marker_features.markers[1].points[0].x = p_edge(0);
    marker_features.markers[1].points[0].y = p_edge(1);
    marker_features.markers[1].points[0].z = p_edge(2);
    marker_features.markers[1].color.r     = 0;
    marker_features.markers[1].color.g     = 1;
    marker_features.markers[1].color.b     = 0;

    marker_features.markers[2].points[0].x = p_corner(0);
    marker_features.markers[2].points[0].y = p_corner(1);
    marker_features.markers[2].points[0].z = p_corner(2);
    marker_features.markers[2].color.r     = 0;
    marker_features.markers[2].color.g     = 0;
    marker_features.markers[2].color.b     = 1;


}

void Publisher::init_surface_marker(const std::string& frame_id){

    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();

    marker_surfaces_array.markers.resize(6 * nbObjects);

    for(std::size_t i = 0; i < marker_surfaces_array.markers.size();i++){

        marker_surfaces_array.markers[i].header.frame_id = frame_id;
        marker_surfaces_array.markers[i].type            = visualization_msgs::Marker::ARROW;
        marker_surfaces_array.markers[i].color.a         = 1.0f;
        marker_surfaces_array.markers[i].color.r         = 1.0f;
        marker_surfaces_array.markers[i].scale.x = 0.025;
        marker_surfaces_array.markers[i].scale.y = 0.025;
        marker_surfaces_array.markers[i].scale.z = 0.025;
        marker_surfaces_array.markers[i].points.resize(2);

    }
}

void Publisher::update_position_test_point(const geo::fCVec3& P){
    marker_test_point.header.stamp = ros::Time::now();
    marker_test_point.points[0].x = P(0);
    marker_test_point.points[0].y = P(1);
    marker_test_point.points[0].z = P(2);
}

void Publisher::update_position_edge(){
    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();
    std::size_t index = 0;
    geo::fCVec3 P1,P2;

    for(std::size_t i=0; i < nbObjects;i++){
        wobj::WBox& wbox = ptr_world_wrapper->get_wbox(i);
        for(std::size_t k = 0; k < 12;k++){

            P1   = wbox.edges[k].P1;
            P2   = wbox.edges[k].P2;

            edge_marker_array.markers[index].header.stamp = ros::Time::now();
            edge_marker_array.markers[index].id = index;
            edge_marker_array.markers[index].points[0].x = P1(0);
            edge_marker_array.markers[index].points[0].y = P1(1);
            edge_marker_array.markers[index].points[0].z = P1(2);

            edge_marker_array.markers[index].points[1].x = P2(0);
            edge_marker_array.markers[index].points[1].y = P2(1);
            edge_marker_array.markers[index].points[1].z = P2(2);
            index++;
        }
    }
}

void Publisher::update_position_surface(){

    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();
    std::size_t index = 0;
    geo::fCVec3 mid,n;
    float scale = 0.1;

    for(std::size_t i=0; i < nbObjects;i++){
        wobj::WBox& wbox = ptr_world_wrapper->get_wbox(i);
        for(std::size_t k = 0; k < 6;k++){

            mid = wbox.surfaces[k].get_middle();
            n   = wbox.surfaces[k].get_normal();

            marker_surfaces_array.markers[index].header.stamp = ros::Time::now();
            marker_surfaces_array.markers[index].id = index;
            marker_surfaces_array.markers[index].points[0].x = mid(0);
            marker_surfaces_array.markers[index].points[0].y = mid(1);
            marker_surfaces_array.markers[index].points[0].z = mid(2);

            marker_surfaces_array.markers[index].points[1].x = scale*n(0) + mid(0);
            marker_surfaces_array.markers[index].points[1].y = scale*n(1) + mid(1);
            marker_surfaces_array.markers[index].points[1].z = scale*n(2) + mid(2);
            index++;
        }

    }

}

void Publisher::update_position_corner(){

    std::size_t nbObjects = ptr_world_wrapper->get_number_objects();
    std::size_t index = 0;


    marker_corners.header.stamp = ros::Time::now();

    for(std::size_t i = 0; i < nbObjects; i++){
            wobj::WBox& wbox = ptr_world_wrapper->get_wbox(i);
            for(std::size_t k = 0; k < 8;k++){


                    marker_corners.points[index].x = wbox.corners[k].C(0);
                    marker_corners.points[index].y = wbox.corners[k].C(1);
                    marker_corners.points[index].z = wbox.corners[k].C(2);

                  /*  << "marker["<<index<<"]: (" << marker_corners.points[index].x << ","
                                                         << marker_corners.points[index].y << ","
                                                         << marker_corners.points[index].z
                                                         << ") " << std::endl;*/


                    index++;
            }

    }
}

void Publisher::update_position_features(){

    p_surface   = ptr_world_wrapper->wrapped_objects.get_closest_point_surface();
    p_edge      = ptr_world_wrapper->wrapped_objects.get_closest_point_edge();
    p_corner    = ptr_world_wrapper->wrapped_objects.get_closest_point_corner();


    marker_features.markers[0].header.stamp = ros::Time::now();
    marker_features.markers[0].points[0].x = p_surface(0);
    marker_features.markers[0].points[0].y = p_surface(1);
    marker_features.markers[0].points[0].z = p_surface(2);
    marker_features.markers[0].id          = 0;


    marker_features.markers[1].header.stamp = ros::Time::now();
    marker_features.markers[1].points[0].x = p_edge(0);
    marker_features.markers[1].points[0].y = p_edge(1);
    marker_features.markers[1].points[0].z = p_edge(2);
    marker_features.markers[1].id          = 1;


    marker_features.markers[2].header.stamp = ros::Time::now();
    marker_features.markers[2].points[0].x = p_corner(0);
    marker_features.markers[2].points[0].y = p_corner(1);
    marker_features.markers[2].points[0].z = p_corner(2);
    marker_features.markers[2].id          = 2;

}

void Publisher::update_position(){
    update_position_corner();
    update_position_surface();
    update_position_edge();

    update_position_features();

}

void Publisher::publish(){
    wrap_pub.publish(marker_corners);
    surface_pub.publish(marker_surfaces_array);
    edge_pub.publish(edge_marker_array);
    test_point_pub.publish(marker_test_point);
    feature_pub.publish(marker_features);

}

}
