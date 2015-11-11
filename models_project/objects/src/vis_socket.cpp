#include "objects/vis_socket.h"

namespace obj{


Vis_socket::Vis_socket(ros::NodeHandle& n,const wobj::WSocket& wsocket):wsocket(wsocket)
{
    socket_pub = n.advertise<visualization_msgs::MarkerArray>("vis_socket", 10);
}

void Vis_socket::initialise(std::size_t num_points, float scale){


    socket_marker_array.markers.resize(4);

    for(std::size_t i = 0; i < socket_marker_array.markers.size();i++){

        socket_marker_array.markers[i].header.frame_id = "/world_frame";
        socket_marker_array.markers[i].type            = visualization_msgs::Marker::LINE_STRIP;
        socket_marker_array.markers[i].id              = i;
        socket_marker_array.markers[i].color.a         = 1.0f;
        socket_marker_array.markers[i].color.r         = 1.0f;
        socket_marker_array.markers[i].color.b         = 1.0f;
        socket_marker_array.markers[i].scale.x         = scale;
        socket_marker_array.markers[i].points.resize(num_points);
    }

    ROS_INFO(" initialise position of socket_markers");


    initialise_disk(socket_marker_array.markers[0].points,wsocket.plate.C,wsocket.plate.R,wsocket.plate.r);
    for(std::size_t i = 0; i < 3;i++){
        initialise_disk(socket_marker_array.markers[i+1].points,wsocket.holes[i].C,wsocket.holes[i].R,wsocket.holes[i].r);
    }

}


void Vis_socket::initialise_disk(std::vector<geometry_msgs::Point>& points,
                                 const geo::fCVec3& C,
                                 const geo::fMat33& R,const float r){

    float degree_dt = 2 * M_PI / (float)points.size();
    geo::fCVec3 X;
    geo::fCVec3 U,V;
    X.zeros();
    float deg = 0;

    std::cout<< "initalise_disk" << std::endl;

    tf::Matrix3x3 rotation;
    geo::fMat33 rot;
    rotation.setEulerZYX(0,M_PI/2,0);

    for(std::size_t i = 0; i < 3;i++){
        for(std::size_t j = 0; j < 3;j++){
            rot(i,j) = rotation[i][j];
        }
    }

     rot.print("tf  Rotation");
     R.print("R rotation");

     U = R.col(0);
     V = R.col(1);


    for(std::size_t i = 0; i < points.size();i++){

            X = C + r * (cos(deg) * U + sin(deg) * V);

            deg = deg + degree_dt;
            points[i].x = X(0);
            points[i].y = X(1);
            points[i].z = X(2);
    }

}


void Vis_socket::publish(){

    for(std::size_t i = 0; i < socket_marker_array.markers.size();i++){
        socket_marker_array.markers[i].header.stamp = ros::Time::now();
    }

    socket_pub.publish(socket_marker_array);
}



}
