#ifndef WSOCKET_H_
#define WSOCKET_H_

#include <string>
#include <array>

#include "geometry.h"

namespace wobj{

class WSocket{

    void setEulerYPR(geo::fMat33& R,float eulerZ, float eulerY,float eulerX)  {
        float ci ( cos(eulerX));
        float cj ( cos(eulerY));
        float ch ( cos(eulerZ));
        float si ( sin(eulerX));
        float sj ( sin(eulerY));
        float sh ( sin(eulerZ));
        float cc = ci * ch;
        float cs = ci * sh;
        float sc = si * ch;
        float ss = si * sh;


        R(0,0) = cj * ch;
        R(0,1) = sj * sc - cs;
        R(0,2) = sj * cc + ss;

        R(1,0) = cj * sh;
        R(1,1) = sj * ss + cc;
        R(1,2) = sj * cs - sc;

        R(2,0) = -sj;
        R(2,1) = cj * si;
        R(2,2) = cj * ci;

    }

public:

    WSocket();

    WSocket(const std::string& name,const  geo::Disk& plate,std::array<geo::Disk,3>& holes);

    void distance_to_features(const geo::fCVec3& P);

    void distance_to_socket_holes(const geo::fCVec3& P);

    void transform(const geo::fCVec3& t, const geo::fCVec3& Rot);

    void transform(const geo::fCVec3& t,float row, float pitch, float yaw);

    const geo::fCVec3& get_edge_projection(const geo::fCVec3& P);

    const geo::fCVec3& get_surface_projection(const geo::fCVec3& P);

    geo::fCVec3& get_edge_projection();

    geo::fCVec3& get_surface_projection();

    void print_info() const;

public:

    float                       dist_surface;
    float                       dist_edge_hole;
    float                       dist_edge_ring;
    float                       dist_edge;
    geo::Disk                   plate;
    geo::fCVec3                 plate_edge_proj;
    geo::fCVec3                 hole_edge_proj;
    std::array<geo::Disk,3>     holes;

private:

    std::string                 name;
    geo::fMat33                 R;
    float                       dist;
    geo::fCVec3                 edge_projection;
    geo::fCVec3                 surface_projection;
    bool                        bInSocket;
    int                         bCount;

};

}


#endif
