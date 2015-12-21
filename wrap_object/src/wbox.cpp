#include "wbox.h"
#include <assert.h>
#include <boost/lexical_cast.hpp>

namespace wobj{

WBox::WBox(){

}

WBox::WBox(const std::string& name,
                    const geo::fCVec3& dim,
                    const geo::fCVec3& origin,
                    const geo::fCVec3& RPY):
name(name),dim(dim),origin(origin)
{
  //  R.zeros();
   // setEulerYPR(R,RPY(2),RPY(1),RPY(0));
  //  set_position(origin,R);

    R.eye();
    set_position(origin,R);


}

void WBox::set_position(const geo::fCVec3& origin, const geo::fMat33& orient){

    getCorners(corners,dim,origin,orient);
    getEdges(edges,corners);
    getSurfaces(surfaces,corners);

    // set surface normals
    geo::fCVec3 n;
    for(std::size_t i = 0; i < 6;i++){
        n = surfaces[i].get_middle() - origin;
        surfaces[i].set_normal(n);
    }
}

void WBox::transform(const geo::fCVec3 &T, const geo::fCVec3 &RPY){
    geo::fMat33 tmp;
    setEulerYPR(tmp,RPY(2),RPY(1),RPY(0));

    R      = R * tmp;
    origin = tmp * origin + T;

    for(std::size_t i=0; i < 8;i++){
        corners[i].transform(T,tmp);
    }
    for(std::size_t i=0; i < 12;i++){
        edges[i].transform(T,tmp);
    }
    // set surface normals
    geo::fCVec3 n;
    for(std::size_t i = 0; i < 6;i++){
        surfaces[i].transform(T,tmp);
        n = surfaces[i].get_middle() - origin;
        surfaces[i].set_normal(n);
    }

}

void WBox::transform(const geo::fCVec3& T){
    assert(false);
/*
    arma::fmat33 tmp;
    tmp.eye();

    R      = R * tmp;
    origin = tmp * origin + T;

    for(std::size_t i=0; i < 8;i++){
        corners[i].transform(T,tmp);
    }
    for(std::size_t i=0; i < 12;i++){
        edges[i].transform(T,tmp);
    }
    // set surface normals
    geo::fCVec3 n;
    for(std::size_t i = 0; i < 6;i++){
        surfaces[i].transform(T,tmp);
        n = surfaces[i].get_middle() - origin;
        surfaces[i].set_normal(n);
    }
*/
}



void WBox::distance_to_features(const geo::fCVec3& P){

    dist.set_max_limit();
    closest_surface_id      = 0;
    closest_edge_id         = 0;
    closest_corner_id       = 0;
    bIsInside               = true;
    std::size_t inside_count = 0;

    for(std::size_t i = 0; i < 6;i++){
        inside[i]   = false;
    }


    // surfaces
    for(std::size_t i = 0; i < nbSurfaces; i++){
        dist_surfaces[i] = surfaces[i].shortest_distance(P);
       if(is_inside(P,i)){
            inside_count++;
       }



        if(std::fabs(dist_surfaces[i]) < dist.min_s){
            dist.min_s = dist_surfaces[i];
            closest_surface_id = i;
        }
    }

    // surfaces
   // distance_to_surfaces(P);

    //edges
    for(std::size_t i = 0; i < nbEdges; i++){
        dist_edges[i] = edges[i].shortest_distance(P);

        if(std::fabs(dist_edges[i]) < dist.min_e){
            dist.min_e = dist_edges[i];
            closest_edge_id = i;
        }
    }

    //corners
    for(std::size_t i = 0; i < nbCorners; i++){
        dist_corners[i] = corners[i].shortest_distance(P);
        if(fabs(dist_corners[i]) < dist.min_c){
            dist.min_c = dist_corners[i];
            closest_corner_id = i;
        }
    }

    if(inside_count == 6){
        bIsInside  = true;
    }else{
        bIsInside  = false;
    }



    dist_surface  =  dist_surfaces[closest_surface_id];
    dist_edge     =  dist_edges[closest_edge_id];
    dist_corner   =  dist_corners[closest_corner_id];
}

float WBox::distance_to_surfaces(const geo::fCVec3& P){

    dist.min_s = std::numeric_limits<float>::max();
    closest_surface_id=0;

    for(std::size_t i = 0; i < nbSurfaces; i++){


        dist_surfaces[i] = surfaces[i].shortest_distance(P);
        if(std::fabs(dist_surfaces[i]) < dist.min_s){
            dist.min_s          = dist_surfaces[i];
            closest_surface_id  = i;
        }
    }
    return dist.min_s;
}

float WBox::distance_to_edges(const geo::fCVec3& P){

    dist.min_e      = std::numeric_limits<float>::max();
    closest_edge_id = 0;

    for(std::size_t i = 0; i < nbEdges; i++){
        dist_edges[i] = edges[i].shortest_distance(P);
        if(dist_edges[i] < dist.min_e){
            dist.min_e      = dist_edges[i];
            closest_edge_id = i;
        }
    }
    return dist.min_e;
}
float WBox::distance_to_edge(const geo::fCVec3& P,std::size_t e_index){
    return  edges[e_index].shortest_distance(P);
}

float WBox::distance_to_surface(const geo::fCVec3& P,std::size_t s_index){
    return  surfaces[s_index].shortest_distance(P);
}

bool WBox::is_inside(const geo::fCVec3 &P){

    // surfaces
    for(std::size_t i = 0; i < nbSurfaces; i++){
        dist_surfaces[i] = surfaces[i].shortest_distance(P);
       if(!is_inside(P,i)){
         return false;
       }
    }
    return true;
}

bool WBox::is_inside(const geo::fCVec3 &P, std::size_t s_index) {
    return surfaces[s_index].is_inside(P);
}

const geo::fCVec3& WBox::get_surface_projection(const geo::fCVec3& P, std::size_t s_index){
    return surfaces[s_index].get_projection(P);
}

const geo::fCVec3& WBox::get_edge_projection(const geo::fCVec3& P,std::size_t e_index){
    return edges[e_index].get_projection(P);
}

geo::fCVec3& WBox::get_edge_projection(){
    return edges[closest_edge_id].get_projection();
}

geo::fCVec3& WBox::get_surface_projection(){
    return surfaces[closest_surface_id].get_projection();
}

geo::fCVec3& WBox::get_corner_projection(){
    return corners[closest_corner_id].get_projection();
}






void WBox::getCorners(std::array<geo::Corner,8>& corners,
                            const geo::fCVec3 &dim,
                            const geo::fCVec3 &origin_,
                            const geo::fMat33& R){

    geo::fCVec3 position;
    position.zeros();
    float l = dim[0];
    float w = dim[1];
    float h = dim[2];
    assert(corners.size() == 8);

    geo::fCVec3 origin = {{origin_[0],origin_[1],origin_[2]}};


    // UP

    position(0) = l/2;    position(1) = w/2;    position(2) = h/2;
    position=position+origin;
    rotate(position,R);

    corners[0].set(position);

    position(0) = l/2;    position(1) = -w/2;    position(2) = h/2;
    position=position+origin;
    rotate(position,R);
    corners[1].set(position);

    position(0) = -l/2;    position(1) = w/2;    position(2) = h/2;
    position=position+origin;
    rotate(position,R);
    corners[2].set(position);

    position(0) = -l/2;    position(1) = -w/2;    position(2) = h/2;
    position=position+origin;
    rotate(position,R);
    corners[3].set(position);

    // DOWN

    position(0) = l/2;    position(1) = w/2;    position(2) = -h/2;
    position=position+origin;
    rotate(position,R);
    corners[4].set(position);

    position(0) = l/2;    position(1) = -w/2;    position(2) = -h/2;
    position=position+origin;
    rotate(position,R);
    corners[5].set(position);

    position(0) = -l/2;    position(1) = w/2;    position(2) = -h/2;
    position=position+origin;
    rotate(position,R);
    corners[6].set(position);

    position(0) = -l/2;    position(1) = -w/2;    position(2) = -h/2;
    position=position+origin;
    rotate(position,R);
    corners[7].set(position);



}

void WBox::getEdges(std::array<geo::Edge,12>& edges, const std::array<geo::Corner,8>& corners){

    // UP
    edges[0] = geo::Edge(corners[0],corners[1]);
    edges[1] = geo::Edge(corners[0],corners[2]);
    edges[2] = geo::Edge(corners[2],corners[3]);
    edges[3] = geo::Edge(corners[3],corners[1]);

    // DOWN
    edges[4] = geo::Edge(corners[4],corners[5]);
    edges[5] = geo::Edge(corners[4],corners[6]);
    edges[6] = geo::Edge(corners[6],corners[7]);
    edges[7] = geo::Edge(corners[7],corners[5]);

    // CONNECTION UP & DOWN
    edges[8]  = geo::Edge(corners[0],corners[4]);
    edges[9]  = geo::Edge(corners[1],corners[5]);
    edges[10] = geo::Edge(corners[2],corners[6]);
    edges[11] = geo::Edge(corners[3],corners[7]);
}


void WBox::getSurfaces(std::array<geo::Surface,6>& surfaces, const std::array<geo::Corner,8> &corners){


    // UP
    surfaces[0] = geo::Surface(corners[0],corners[1],corners[2],corners[3]);

    // DOWN

    surfaces[1] = geo::Surface(corners[4],corners[5],corners[6],corners[7]);

    // SIDES

    surfaces[2] = geo::Surface(corners[0],corners[1],corners[5],corners[4]);
    surfaces[3] = geo::Surface(corners[0],corners[2],corners[6],corners[4]);
    surfaces[4] = geo::Surface(corners[2],corners[3],corners[7],corners[6]);
    surfaces[5] = geo::Surface(corners[1],corners[3],corners[7],corners[5]);
}

void WBox::translate(geo::fCVec3& target,const geo::fMat33& R,const geo::fCVec3& T){
    target = R * target + T;
}

void WBox::rotate(geo::fCVec3& target,const geo::fMat33& R){
    target = R * target;
}

void WBox::print_info() const{
    std::cout<< "=== WBox: " << name << " === " << std::endl;
    std::cout<< "  -- corners -- " << std::endl;
    for(std::size_t j = 0; j < corners.size();j++){

        corners[j].print(boost::lexical_cast<std::string>(j));
    }

   /* std::cout<< " -- edges -- " << std::endl;
    for(std::size_t i = 0; i < edges.size();i++){
        edges[i].print();
    }
    std::cout<< " --------- " << std::endl;*/

   /* std::cout<< " -- surfaces -- " << std::endl;
    for(std::size_t i = 0; i < surfaces.size();i++){
        surfaces[i].print();
    }*/
}


}
