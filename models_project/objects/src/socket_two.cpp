#include "objects/socket_two.h"

namespace obj{


Socket_two::Socket_two(std::string link_socket_name,
                       std::string link_box_name,
                       const tf::Vector3& origin,
                       const tf::Vector3 &rpy, float scale){


    //const std::string& name, const geo::fCVec3& C, const geo::fMat33& R, float r
    geo::fMat33 R;
    R.eye();

    disk_radius = 0.02  * scale;
    hole_radius = 0.002 * scale;

    std::array<geo::Disk,3> holes;
    geo::fCVec3 position;
    position.zeros();

    geo::Disk plate("plate",position,R,disk_radius);


    position.zeros();
    position(0) =   -0.01 * scale;
    holes[0]    = geo::Disk("hole_left",position,R,hole_radius);

    position.zeros();
    position(0) =    0.01 * scale;
    holes[1]    = geo::Disk("hole_right",position,R,hole_radius);

    position.zeros();
    position(1)     =  0.005 * scale;
    holes[2]        = geo::Disk("hole_up",position,R,hole_radius);

    wsocket = wobj::WSocket(link_socket_name,plate,holes);


    geo::fCVec3 dim             = {{0.06,0.06,0.03}};
                dim             = dim * scale;
    geo::fCVec3 orientation     = {{0,0,0}};

    position.zeros();
    position(2) = -0.005/2 - (0.03 - 0.005)/2;
    wbox = wobj::WBox(link_box_name,dim,position,orientation);

    // create boxes to represent the edges of the socket // usefull for the filter
    {
        // dim(0) is x direction
        geo::fCVec3 dim_edgeb; dim_edgeb(0) =  dim(0) + 0.06; dim_edgeb(1) = 0.03; dim_edgeb(2) = 0.06;
        position.zeros();
        position(1) = dim(0)/2 + 0.015;
        edge_wboxes[0] = wobj::WBox("socket_top_edge",dim_edgeb,position,orientation);
    }
    {
        geo::fCVec3 dim_edgeb; dim_edgeb(0) =  dim(0) + 0.06; dim_edgeb(1) = 0.03 ; dim_edgeb(2) = 0.06;
        position.zeros();
        position(1) = -dim(0)/2 - 0.015;
        edge_wboxes[1] = wobj::WBox("socket_bottom_edge",dim_edgeb,position,orientation);
    }

    {
        geo::fCVec3 dim_edgeb; dim_edgeb(0) =  0.03; dim_edgeb(1) = dim(0) + 0.06; dim_edgeb(2) = 0.06;
        position.zeros();
        position(0) = -dim(0)/2 - 0.015;
        edge_wboxes[2] = wobj::WBox("socket_left_edge",dim_edgeb,position,orientation);
    }

    {
        geo::fCVec3 dim_edgeb; dim_edgeb(0) =  0.03; dim_edgeb(1) = dim(0) + 0.06; dim_edgeb(2) = 0.06;
        position.zeros();
        position(0) =  dim(0)/2 + 0.015;
        edge_wboxes[3] = wobj::WBox("socket_right_edge",dim_edgeb,position,orientation);
    }

    // create boxes to represent the socket holes (instead of cylinders )

    double dx = 0.001;

    position.zeros();
    position(0) =   -0.01 * scale;
    dim(0)         = 0.01;
    dim(1)         = 0.01;
    dim(2)         = 0.05;
    position(2) = -0.005/2 - (dim(2) - 0.005)/2 + dx;

    hole_wboxes[0] = wobj::WBox("box_hole_1",dim,position,orientation);

    position.zeros();
    position(0)    =    0.01 * scale;
    position(2) = -0.005/2 - (dim(2)- 0.005)/2 + dx;
    hole_wboxes[1] = wobj::WBox("box_hole_2",dim,position,orientation);

    position.zeros();
    position(1)     =  0.005 * scale;
    position(2) = -0.005/2 - (dim(2) - 0.005)/2 + dx;
    hole_wboxes[2] = wobj::WBox("box_hole_3",dim,position,orientation);

    // Transformation

    orientation(0) = rpy.x();
    orientation(1) = rpy.y();
    orientation(2) = rpy.z();

    geo::fCVec3 T; T(0) = origin.x(); T(1)  = origin.y(); T(2)  = origin.z();

    wsocket.transform(T,orientation);
    wbox.transform(T,orientation);

    hole_wboxes[0].transform(T,orientation);
    hole_wboxes[1].transform(T,orientation);
    hole_wboxes[2].transform(T,orientation);

    edge_wboxes[0].transform(T,orientation);
    edge_wboxes[1].transform(T,orientation);
    edge_wboxes[2].transform(T,orientation);
    edge_wboxes[3].transform(T,orientation);



}


}



