#include "wsocket.h"
#include <limits>

namespace wobj{

WSocket::WSocket(){

}

WSocket::WSocket(const std::string& name,const  geo::Disk& plate,std::array<geo::Disk,3>& holes)
:name(name),plate(plate),holes(holes)
{

}

void WSocket::distance_to_features(const geo::fCVec3& P){

    dist_edge = std::numeric_limits<float>::max();
    bInSocket=false;
    bCount = 0;

    plate.shortest_distance(P);

    surface_projection  = plate.get_projection();
    edge_projection     = plate.K;
    dist_edge           = plate.dist_edge;
    dist_surface        = plate.dist_surface;

   for(std::size_t i = 0; i < 3;i++){

        holes[i].shortest_distance(P);
        dist   = holes[i].dist_edge;

        if(dist < dist_edge){
            dist_edge       = dist;
            edge_projection = holes[i].K;
        }

        if(holes[i].bIn){
            surface_projection = edge_projection;
            i = 3;
        }
    }
}


void WSocket::distance_to_socket_holes(const geo::fCVec3& P){

}

void WSocket::transform(const geo::fCVec3& t,float row, float pitch, float yaw){
    geo::fCVec3 Rot = {{row,pitch,yaw}};
    transform(t,Rot);
}

void WSocket::transform(const geo::fCVec3& t, const geo::fCVec3& Rot){

    setEulerYPR(R,Rot(2),Rot(1),Rot(0));

    std::cout<< "apply transformation" << std::endl;
    t.print("translation");
    R.print("rotation");

    plate.transform(t,R);
    holes[0].transform(t,R);
    holes[1].transform(t,R);
    holes[2].transform(t,R);

}

const geo::fCVec3& WSocket::get_edge_projection(const geo::fCVec3& P){
    distance_to_features(P);
    return edge_projection;
}

const geo::fCVec3 &WSocket::get_surface_projection(const geo::fCVec3 &P){
    distance_to_features(P);
    return surface_projection;
}

geo::fCVec3& WSocket::get_edge_projection(){
    return edge_projection;
}

geo::fCVec3& WSocket::get_surface_projection(){
    return surface_projection;
}

void WSocket::print_info() const{

    std::cout<< " === WSocket: " << name << " === " << std::endl;
    std::cout<< "--- plate ---" << std::endl;
    plate.print();
    for(std::size_t i = 0; i < 3;i++){
        std::cout<< " --- hole(" <<i<<") --- " << std::endl;
        holes[i].print();
    }

}

}
