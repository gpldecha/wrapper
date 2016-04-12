#ifndef WRAPOBJECT_H_
#define WRAPOBJECT_H_

// STL

#include <algorithm>    // std::sort
#include <array>
#include <string>
#include <map>
#include <array>

#include "geometry.h"
#include "wbox.h"
#include "wsocket.h"
#include "spacepartition.h"

namespace wobj{

typedef std::array<float,3> Vector3;

enum feature_type {EDGE=0,CORNER=1,SURFACE=2};
enum object_type {BOX=0,CIRCLE=1,SOCKET=2};

class distances2{
public:
    distances2(){
        min_e = min_c = min_s = 0;
    }

    void set_max_limit(){
        min_e =  std::numeric_limits<float>::max();
        min_c = min_s = min_e;
    }

    object_type min_s_obj_type;
    object_type min_e_obj_type;
    object_type min_c_obj_type;

    std::size_t min_e_id;
    std::size_t min_c_id;
    std::size_t min_s_id;


    float min_e;
    float min_s;
    float min_c;
};

typedef struct {
    object_type     obj_type;
    feature_type    fea_type;
    std::size_t     index_obj;
    std::size_t     index_fea;
} ID_object;

typedef enum print_info_type{
    NAMES,
    ALL
} print_info_type;


class WrapObject{


public:

    WrapObject();

    void push_back_box(WBox *wbox);

    void push_back_circle(const std::string& name, const geo::fCVec3 &C, const geo::fCVec3& R, float r);

    void push_back_circle(const std::string& name, const geo::fCVec3 &C, const geo::fMat33& R, float r);

    void push_back_socket(const std::string &name, const geo::Disk& plate, std::array<geo::Disk,3> &holes);

    void push_back_socket(const wobj::WSocket& wsocket);

    ///
    /// \brief distance_to_features, computes the closest distance and positions of the box's features
    ///        (edge,corner,surface) from point P
    /// \param P
    ///
    void distance_to_features(const geo::fCVec3& P);

    void set_origin_orientation(const std::string& name,const geo::fCVec3& origin, const geo::fCVec3& orientation);


    float get_distance_to_corner();
    float get_distance_to_edge();
    float get_distance_to_surface();

    geo::fCVec3& get_closest_point_edge();
    geo::fCVec3& get_closest_point_surface();
    geo::fCVec3& get_closest_point_corner();

    ///
    /// \brief get_closest_surface
    /// \return ID of closest surface
    ///
    void get_closest_surface(const geo::fCVec3& P, ID_object &id_object);

    void get_closest_edge(const geo::fCVec3& P, ID_object &ID_object);

    float distance_to_surface(const geo::fCVec3& P, const ID_object &id_object);

    float distance_to_edge(const geo::fCVec3& P, const ID_object &id_object);

    const geo::fCVec3& get_surface_projection(const geo::fCVec3& P, const ID_object &id_object);

    const geo::fCVec3& get_edge_projection(const geo::fCVec3& P, const ID_object& id_object);

    bool is_inside_surface(const geo::fCVec3 &P, std::size_t object_index, std::size_t surface_index);

    bool is_inside_box();

    std::string get_wobject_name(std::size_t i) const;

    int get_wobject_index(const std::string& name) const;

    std::size_t get_number_objects() const;

    wobj::WBox &get_wbox(std::string name);

    wobj::WBox &get_wbox(std::size_t index);

    void print_points(const std::string& name);

    void print_info(print_info_type type) const;

private:

    void  inline check_closest_feature_wbox(std::size_t wbox_index,float dist_surf,float dist_edge,float dist_corner){
        if(dist_surf < dist.min_s){
            dist.min_s          = dist_surf;
            dist.min_s_id       = wbox_index;
            dist.min_s_obj_type = BOX;
        }
        if(dist_edge < dist.min_e){
            dist.min_e         = dist_edge;
            dist.min_e_id      = wbox_index;
            dist.min_e_obj_type = BOX;

        }
        if(dist_corner < dist.min_c){
            dist.min_c          = dist_corner;
            dist.min_c_id       = wbox_index;
            dist.min_c_obj_type = BOX;
        }
    }

    void inline check_closest_feature_wsocket(float dist_surf,float dist_edge){
        /*if(dist_surf < dist.min_s){
            dist.min_s          = dist_surf;
            dist.min_s_obj_type = SOCKET;
        }*/
        if(dist_edge < dist.min_e){
            dist.min_e         = dist_edge;
            dist.min_e_obj_type = SOCKET;
        }
    }

    void set_distance_and_projections();

public:

    std::vector<WBox*>                  wboxes;     /// list of wrapped boxes
    std::vector<geo::Circle>            wcircles;   /// list of circles
    WSocket                             wsocket;    /// socket

private:


    bool                                bIsInside;
    bool                                bSocket;
    bool                                bWBox;

    distances2                          dist;

    geo::fCVec3                         closest_point_surface;
    geo::fCVec3                         closest_point_edge;
    geo::fCVec3                         closest_point_corner;

    std::map<std::string,std::size_t>   name_index;
    std::map<std::string,std::size_t>::iterator name_it;



};

}





#endif
