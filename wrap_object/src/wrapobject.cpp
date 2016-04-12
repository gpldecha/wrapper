#include "wrapobject.h"
#include <iostream>
#include <limits>
#include <boost/lexical_cast.hpp>
#include <algorithm>    // std::sort
#include <cmath>
#include "geometry.h"

using namespace geo;


namespace wobj{

WrapObject::WrapObject()
{

    wboxes.clear();
    bSocket = false;
    bWBox   = false;
}

void WrapObject::push_back_box(wobj::WBox* wbox){
    wboxes.push_back(wbox);
    std::cout<< "push back: " << wbox->name << std::endl;
    name_index[wbox->name] = wboxes.size()-1;
    bWBox   = true;
}

void WrapObject::push_back_circle(const std::string& name, const geo::fCVec3 &C, const geo::fCVec3& orienation, float r){

    geo::fMat33 R;
    rotation_matrix(R,orienation);

    wcircles.push_back(geo::Circle(name,C,R,r));

}

void WrapObject::push_back_circle(const std::string& name, const geo::fCVec3 &C, const geo::fMat33& R, float r){
   wcircles.push_back(geo::Circle(name,C,R,r));
}

void WrapObject::push_back_socket(const std::string& name,const  geo::Disk& plate,std::array<geo::Disk,3>& holes){
    wsocket =  WSocket(name,plate,holes);
    bSocket = true;
}

void WrapObject::push_back_socket(const wobj::WSocket& wsocket){
    this->wsocket = wsocket;
    bSocket = true;
}

bool WrapObject::is_inside_box(){
    return bIsInside;
}

void WrapObject::distance_to_features(const fCVec3 &P){

    dist.set_max_limit();
    bIsInside   = false;

    std::size_t hack = 2;


    // Check closest features box
    if(bWBox){
        for(std::size_t i = 0; i < wboxes.size();i++){
           // std::cout<< "wboxes["<<i<<"]->distance_to_features(P)" << std::endl
            wboxes[i]->distance_to_features(P);
            check_closest_feature_wbox(i, wboxes[i]->dist_surface, wboxes[i]->dist_edge,wboxes[i]->dist_corner);
            // if inside of any of the boxes return true;
            if(wboxes[i]->bIsInsideWbox){
                bIsInside = true;
            }
        }
    }

    // Check closest features socket
  /*  if(bSocket){
        wsocket.distance_to_features(P);
        check_closest_feature_wsocket(wsocket.dist_surface,wsocket.dist_edge);
    }

    // Check closest feature circle
    float tmp;
    for(std::size_t i = 0; i < wcircles.size();i++){
        tmp = wcircles[i].shortest_distance(P);
        if(tmp < dist.min_e){
            dist.min_e          = tmp;
            dist.min_e_id       = i;
            dist.min_e_obj_type = CIRCLE;
        }
     }
        */
    set_distance_and_projections();

}

void WrapObject::get_closest_surface(const fCVec3 &P,ID_object& id_object){

    dist.set_max_limit();
    float          dist_surf;
    std::size_t    closest_surface_id       = 0;
    std::size_t    closet_wbox              = 0;

    std::size_t hack = 2;

    if(bWBox){
        for(std::size_t i = 0; i < hack/*wboxes.size()*/;i++){
            dist_surf         = wboxes[i]->distance_to_surfaces(P);
            if(dist_surf < dist.min_s){
                dist.min_s                  = dist_surf;
                dist.min_s_obj_type         = BOX;
                closest_surface_id          = wboxes[i]->closest_surface_id;
                closet_wbox                 = i;

            }
        }
    }

    id_object.obj_type  = dist.min_s_obj_type;
    id_object.fea_type  = SURFACE;
    id_object.index_obj = closet_wbox;
    id_object.index_fea = closest_surface_id;
}

void WrapObject::get_closest_edge(const fCVec3 &P, ID_object &id_object){

    dist.min_e                              = std::numeric_limits<float>::max();
    float          dist_edge;
    std::size_t    closest_edge_id          = 0;
    std::size_t    closet_edge_object_id    = 0;
    std::size_t    hack                     = 2;
 //  P.print("P");
    if(bWBox){
        for(std::size_t i = 0; i < hack /*wboxes.size()*/;i++){
            dist_edge         = wboxes[i]->distance_to_edges(P);
      //      std::cout<< "box("<<i<<"): " << dist_edge << std::endl;
            if(dist_edge < dist.min_e){
                dist.min_e                  = dist_edge;
                dist.min_e_obj_type         = BOX;
                closest_edge_id             = wboxes[i]->closest_edge_id;
                closet_edge_object_id                 = i;
            }
        }
    }
    if(bSocket){
        wsocket.distance_to_features(P);
       // std::cout<< "wsk(0): " << wsocket.dist_edge << std::endl;

       if(wsocket.dist_edge < dist.min_e){
            dist.min_e                  = wsocket.dist_edge;
            dist.min_e_obj_type         = SOCKET;
            closest_edge_id             = 0;
            closet_edge_object_id       = 0;
        }
    }

  //  std::cout<< "min_dist: " << dist.min_e << std::endl;

    id_object.obj_type  = dist.min_e_obj_type;
    id_object.fea_type  = EDGE;
    id_object.index_obj = closet_edge_object_id;
    id_object.index_fea = closest_edge_id;

}


float WrapObject::distance_to_surface(const geo::fCVec3& P,const ID_object &id_object){

    switch (id_object.obj_type) {
    case BOX:
    {
        return  wboxes[id_object.index_obj]->distance_to_surface(P,id_object.index_fea);
        break;
    }
    case SOCKET:
    {
        return wsocket.dist_surface;
        break;
    }
    default:
    {
        return -1;
        break;
    }
    }
}


float WrapObject::distance_to_edge(const geo::fCVec3& P,const ID_object &id_object){

    switch (id_object.obj_type) {
    case BOX:
    {
        return  wboxes[id_object.index_obj]->distance_to_edge(P,id_object.index_fea);
        break;
    }
    case SOCKET:
    {
        wsocket.distance_to_features(P);
        return wsocket.dist_edge;
        break;
    }
    default:
    {
        return -1;
        break;
    }
    }
}


const geo::fCVec3& WrapObject::get_surface_projection(const geo::fCVec3& P, const ID_object &id_object){

    switch (id_object.obj_type) {
    case BOX:
    {
        return  wboxes[id_object.index_obj]->get_surface_projection(P,id_object.index_fea);
        break;
    }
    case SOCKET:
    {
        return wsocket.get_surface_projection(P);
        break;
    }
    default:
    {
        return  wboxes[0]->get_edge_projection(P,0);
        break;
    }
    }
}

const geo::fCVec3& WrapObject::get_edge_projection(const geo::fCVec3& P, const ID_object& id_object){
    switch (id_object.obj_type) {
    case BOX:
    {
        return  wboxes[id_object.index_obj]->get_edge_projection(P,id_object.index_fea);
        break;
    }
    case SOCKET:
    {
        return wsocket.get_edge_projection(P);
        break;
    }
    default:
    {
        return  wboxes[0]->get_edge_projection(P,0);
        break;
    }
    }
}


bool WrapObject::is_inside_surface(const geo::fCVec3& P,std::size_t object_index,std::size_t surface_index){
    return wboxes[object_index]->is_inside(P,surface_index);
}

void WrapObject::set_distance_and_projections(){
    closest_point_edge.zeros();
    closest_point_corner.zeros();
    closest_point_surface.zeros();

    // check which object has the closest SURFACE
    switch(dist.min_s_obj_type){
    case BOX:
    {
        if(bWBox){
            closest_point_surface = wboxes[dist.min_s_id]->get_surface_projection();
        }
        break;
    }
    case CIRCLE:
    {
        /// no surface for a circle
        break;
    }
    case SOCKET:
    {
        //closest_point_surface =   wsocket.get_surface_projection();
        break;
    }
    }


    // check which object has the closest EDGE
    switch(dist.min_e_obj_type){
    case BOX:
    {
        if(bWBox){
            closest_point_edge = wboxes[dist.min_e_id]->get_edge_projection();
        }
        break;
    }
    case CIRCLE:
    {
        closest_point_edge = wcircles[dist.min_e_id].get_projection();
        break;
    }
    case SOCKET:
    {
        closest_point_edge =   wsocket.get_edge_projection();
        break;
    }
    }

    // check which object has the closest CORNER
    if(wboxes.size() != 0){
        closest_point_corner = wboxes[dist.min_c_id]->get_corner_projection();
    }

}

float WrapObject::get_distance_to_corner(){
    return dist.min_c;
}

float WrapObject::get_distance_to_edge(){
    return dist.min_e;
}

float WrapObject::get_distance_to_surface(){
    return dist.min_s;
}

geo::fCVec3& WrapObject::get_closest_point_edge(){
    return closest_point_edge;
}

geo::fCVec3& WrapObject::get_closest_point_surface(){
    return closest_point_surface;
}

geo::fCVec3& WrapObject::get_closest_point_corner(){
    return closest_point_corner;
}


std::string WrapObject::get_wobject_name(std::size_t i) const{
    if(i < wboxes.size()){
        return wboxes[i]->name;
    }else{
        return "no such index: " + boost::lexical_cast<std::string>(i);
    }
}

int WrapObject::get_wobject_index(const std::string& name) const{
   for(std::size_t i = 0; i < wboxes.size();i++){
        if(wboxes[i]->name == name){
            return i;
        }
    }
    return -1;
}


void WrapObject::set_origin_orientation(const std::string &name, const fCVec3 &origin, const fCVec3 &orientation){
   // wboxes[name_index[name]].transform(origin,orientation);
}

std::size_t WrapObject::get_number_objects() const{
    return wboxes.size();
}

wobj::WBox &WrapObject::get_wbox(std::string name){

   //for(name_it == name_index.begin(); name_it != name_index.end();name_it++){
       // std::cout<< "names: " << (name_it->first) << std::endl;

   // }

    if( name_index.find(name) != name_index.end()){
        return *wboxes[name_index[name]];
    }else{
        std::cout<< "no such wbox present: " << name << std::endl;
        exit(0);
        return *wboxes[0];
    }

}

wobj::WBox &WrapObject::get_wbox(std::size_t index){
    return *wboxes[index];
}

void WrapObject::print_points(const std::string& name){

    /*if(name == "corners"){
        for(std::size_t i = 0; i < wboxes.size();i++){
             for(std::size_t j = 0; j < wboxes[i].corners.size(); j++){
                    wboxes[i].corners[j].print();
             }
        }
    }else if(name == "edges"){
        for(std::size_t i = 0; i < wboxes.size();i++){
             for(std::size_t j = 0; j < wboxes[i].edges.size(); j++){
                    //wboxes[i].edges[j].print();
             }
        }

    }else if(name == "surfaces"){
        for(std::size_t i = 0; i < wboxes.size();i++){
             for(std::size_t j = 0; j < wboxes[i].surfaces.size(); j++){
                    wboxes[i].surfaces[j].print();
             }
        }
    }else{
        std::cout<< "ERROR: " << name << " does not excist, current options are [corners,edges,surfaces]" << std::endl;
    }*/

}


void WrapObject::print_info(print_info_type type) const{
    switch(type){
    case ALL:
    {
        for(std::size_t i = 0; i < wboxes.size();i++){
            wboxes[i]->print_info();
        }
        break;
    }
    case NAMES:
    {
        for(std::size_t i = 0; i < wboxes.size();i++){
           std::cout<< "wboxes[" << i << "]: " << wboxes[i]->name << std::endl;
        }

        break;
    }
    default:
    {
        break;
    }
    }

}


}
