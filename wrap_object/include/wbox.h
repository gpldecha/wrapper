#ifndef WBOX_H_
#define WBOX_H_

#include "geometry.h"
#include <array>

namespace wobj{

inline void rotation_matrix(geo::fMat33& R, const geo::fCVec3 &orientation){
    float a = orientation(0);
    float b  = orientation(1);
    float g = orientation(2);


    R(0,0) = cos(a) * cos(b);
    R(0,1) = cos(a) * sin(b) * sin(g) - sin(a)*cos(g);
    R(0,2) = cos(a) * sin(b) * cos(g) + sin(a) *sin(g);

    R(1,0) = sin(a) * cos(b);
    R(1,1) = sin(a) * sin(b) *sin(g) + cos(a) * cos(g);
    R(1,2) = sin(a)*sin(b)*cos(g) -cos(a)*sin(g);

    R(2,0) = -sin(b);
    R(2,1) = cos(b) * sin(g);
    R(2,2) = cos(b) * cos(g);

}


class distances{
public:
    distances(){
        min_e = min_c = min_s = 0;
    }

    void set_max_limit(){
        min_e =  std::numeric_limits<float>::max();
        min_c = min_s = min_e;
    }

    float min_e;
    float min_s;
    float min_c;
};


class WBox{

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

    WBox();

    WBox(const std::string& name, const geo::fCVec3& dim, const geo::fCVec3& origin, const geo::fCVec3& RPY);

    void distance_to_features(const geo::fCVec3& P);

    float distance_to_surfaces(const geo::fCVec3& P);

    float distance_to_surface(const geo::fCVec3& P,std::size_t s_index);

    float distance_to_edges(const geo::fCVec3& P);

    float distance_to_edge(const geo::fCVec3& P,std::size_t e_index);

    void transform(const geo::fCVec3& T);

    void transform(const geo::fCVec3& T, const geo::fCVec3& RPY);

    const geo::fCVec3& get_surface_projection(const geo::fCVec3& P,std::size_t s_index);
    const geo::fCVec3& get_edge_projection(const geo::fCVec3& P, std::size_t e_index);

    const distances &get_distances() const;

    geo::fCVec3& get_edge_projection();
    geo::fCVec3& get_surface_projection();
    geo::fCVec3& get_corner_projection();

    void print_info() const;

    bool is_inside() const;

    bool is_inside(const geo::fCVec3 &P);

    bool is_inside(const geo::fCVec3& P,std::size_t s_index);


private:


    void set_position(const geo::fCVec3& origin, const geo::fMat33& orient);

    void getCorners(std::array<geo::Corner,8>& corners, const geo::fCVec3 &dim, const geo::fCVec3 &origin, const geo::fMat33 &R);

    void getEdges(std::array<geo::Edge,12>& edges, const std::array<geo::Corner,8>& corners);

    void getSurfaces(std::array<geo::Surface,6>& surfaces,const std::array<geo::Corner,8>& corners);

     void translate(geo::fCVec3& target,const geo::fMat33& R,const geo::fCVec3& T);

     void rotate(geo::fCVec3& target,const geo::fMat33& R);


public:

    float           dist_surface;
    float           dist_edge;
    float           dist_corner;
    bool            bIsInsideWbox;
    std::string     name;

    std::array<geo::Corner,8>        corners;
    std::array<geo::Edge,12>         edges;
    std::array<geo::Surface,6>       surfaces;

    std::size_t                     closest_surface_id;
    std::size_t                     closest_edge_id;
    std::size_t                     closest_corner_id;



    geo::fCVec3         origin;
    geo::fMat33         R;
    geo::fCVec3         dim;

    private:

  //  std::array<bool,6>       inside;


    const static std::size_t nbEdges    = 12;
    const static std::size_t nbCorners  =  8;
    const static std::size_t nbSurfaces =  6;


    std::array<float,nbSurfaces>    dist_surfaces;
    std::array<float,nbCorners>     dist_corners;
    std::array<float,nbEdges>       dist_edges;

    distances dist;


    geo::fCVec3     proj_surface;
    geo::fCVec3     proj_edge;
    geo::fCVec3     proj_corner;


};

}


#endif
