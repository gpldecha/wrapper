#include "geometry.h"
#include <assert.h>

namespace geo{


Circle::Circle(){

}

Circle::Circle(const std::string &name, const fCVec3 &C, const fMat33& R, float r):
name(name),C(C),R(R),r(r)
{
    U         = R.col(0);
    V         = R.col(1);
    N         = R.col(2);

    invR      = R.st();
    invC      = -invR * C;
}

void Circle::print() const{
    C.print("C");
    N.print("N");
    U.print("U");
    V.print("V");
    std::cout<< "r: " << r << std::endl;
}

geo::fCVec3 &Circle::get_projection(){
    return K;
}

void Circle::transform(const geo::fCVec3& t,const geo::fMat33& Rot){
    R         = Rot * R;
    C         = R * C + t;
    U         = R.col(0);
    V         = R.col(1);
    N         = R.col(2);
    invR      = R.st();
    invC      = -invR * C;
}

float Circle::shortest_distance(const fCVec3 &P_w){
    P_c = invR* P_w + invC;//P_w - C;
    K   = C + r * (P_c(0)/sqrt(P_c(0)*P_c(0) + P_c(1)*P_c(1)) * U + P_c(1)/sqrt(P_c(0)*P_c(0) + P_c(1)*P_c(1)) * V);
    return arma::norm(K - P_w,2);
}


Disk::Disk():Circle(){}

Disk::Disk(const std::string& name, const geo::fCVec3& C, const geo::fMat33& R, float r):
    Circle(name,C,R,r)
{
}


geo::fCVec3 &Disk::get_projection(){
    if(arma::norm(Q - C,2) <= r){
        return Q;
    }else{
        return K;
    }
}

float Disk::shortest_distance(const geo::fCVec3& P_w){
   // std::cout << "Disk::shortest_distance" << std::endl;
    P_c =  invR* P_w + invC;
    Q = P_w - arma::dot(N,P_w - C) * N;

    float q_u = P_c(0)/sqrt(P_c(0)*P_c(0) + P_c(1)*P_c(1));
    float q_v = P_c(1)/sqrt(P_c(0)*P_c(0) + P_c(1)*P_c(1));

  //  std::cout<< "q_u: " << q_u << "   q_v: " << q_v << std::endl;
    K   = C + r * (q_u * U + q_v * V);

    dist_edge       = arma::norm(K - P_w,2);
    dist_surface    = arma::norm(Q - P_w,2);

    if(arma::norm(Q - C,2) <= r){
        bIn = true;
        return dist_surface;
    }else{
        bIn = false;
        return dist_edge;
    }
}


// ---------------------------------- Cylinder --------------------------------------------

Cylinder::Cylinder(){}

Cylinder::Cylinder(const std::string name, const geo::fCVec3& x_ref, const geo::fMat33& R, float radius,float length):
    name(name),x_ref(x_ref),R(R),radius(radius),length(length)
{
    RT = R.st();
}

void Cylinder::projection(geo::fCVec3& P)
{
/** PROJECTION, project a set of points onto a a cylinder
    %
    %   input ------------------------------------------------
    %
    %       o P: (3 x 1), cartesian point
    %
    %   output ------------------------------------------------
    %
    %       o proj: (3 x 1), projected point
    %
   **/

    //  X in the framce of refrence of the cylinder

    // project P onto frame of reference of X

    R.print("R");
    x_ref.print("x_ref");


    X_c  = RT * P - RT * x_ref;

    X_c.print("X_c");

    // Z-axis projection (along length of cylinder)
   /* p_proj(2)  = X_c(2);
    if(p_proj(2) > length){
       p_proj(2) = length;
    }
    if(p_proj(2) < 0)
    {
        p_proj(2) = 0;
    }*/

     tmp(0) = X_c(0);
     tmp(1) = X_c(1);
    // (N x 2) vector pointing from cylinder origin to point
    tmp = tmp / arma::norm(tmp);


    p_proj(0) = tmp(0) * radius;
    p_proj(1) = tmp(1) * radius;

    p_proj = R * p_proj + x_ref;

}

bool Cylinder::is_inside(geo::fCVec3& P){

    X_c = R.st() * P - R.st() * x_ref;

    if(X_c(2) < length && X_c(3) >= 0){

        if (arma::norm(X_c(arma::span(0,1)),2) <= radius){
          return true;
        }else{
          return false;
        }
    }else{
        return false;
    }
}


Geometry::~Geometry(){
   projection.zeros();
}

double Geometry::distance_point_line_segment(const fCVec3 &P1, const fCVec3 &P2, const fCVec3 &P) {

    v = (P2 - P1);
    l =  arma::norm(v,2);
    if (l == 0.0){
        if(isBigger(P1,P2)){
            projection = P1;
        }else{
            projection = P2;
        }
        return 0;   // v == w case
    }

    v       = v/l;
    t = arma::dot(P - P1,v);

    if(t >= l || t < 0){

        //check witch point edge semgment lies closest
        if( arma::norm(P - P1,2) <= arma::norm(P - P2,2) ){
                projection = P1;
        }else{
                projection = P2;
        }
    }else{
        projection = P1 + (v*t);  // Projection falls on the segment
    }

    return arma::norm(P - projection,2);
}

bool Geometry::isBigger(const fCVec3& v1,const fCVec3& v2){

    if(v1(0) >= v2(0) && v1(1) >= v2(1) && v1(2) >= v2(2)){
        return true;
    }else{
        return false;
    }

}

std::array<float,3> &Geometry::GetProjection(){
    proj[0] = projection(0);
    proj[1] = projection(1);
    proj[2] = projection(2);
    return proj;
}

geo::fCVec3 &Geometry::get_projection(){
    return projection;
}


void Surface::init(){
    u.zeros();
    v.zeros();
    s = t = 0;
    n.zeros();
    w.zeros();
    projection.zeros();
    V0.zeros(); V1.zeros(); V2.zeros(); V3.zeros();
    VMiddle.zeros();
}


Surface::Surface(){
    init();
}

Surface::Surface(const Corner& C0,const Corner& C1,const Corner& C2,const Corner& C3){
    init();

    V0 = C0.C;
    V1 = C1.C;
    V2 = C2.C;
    V3 = C3.C;
    reset_variables();
}

void Surface::reset_variables(){

    VMiddle = (V0 + V1 + V2 + V3)/4;

    // find the neighbours of V0

    arma::vec tmp(3);
    tmp(0) = std::sqrt(arma::sum(arma::pow(V1 - V0,2)));
    tmp(1) = std::sqrt(arma::sum(arma::pow(V2 - V0,2)));
    tmp(2) = std::sqrt(arma::sum(arma::pow(V3 - V0,2)));
    arma::uvec indices = arma::sort_index(tmp);

    if(indices(0) == 0)
    {
        u = V1 - V0;
    }else if(indices(0) == 1){
        u = V2 - V0;
    }else{
        u = V3 - V0;
    }

    if(indices(1) == 0)
    {
        v = V1 - V0;
    }else if(indices(1) == 1){
        v = V2 - V0;
    }else{
        v = V3 - V0;
    }

    assert(arma::sum(v - u) != 0);

    s = t = 0;
    n = arma::cross(u,v);
    n = arma::normalise(n);
    d = - arma::dot(n,V0);
}


double Surface::shortest_distance(const fCVec3 &P){
    w = P - V0;
    projected_P = P - n * arma::dot(w,n);
    if(arma::dot(w,n) < 0){
        isInside = true;
    }else{
        isInside = false;
    }
    //projection = projected_P;
    compute_projection_on_plane_segement();
    return arma::norm(projection - P,2);
}

const geo::fCVec3& Surface::get_projection(const geo::fCVec3& P){
    shortest_distance(P);
    return projection;
}

geo::fCVec3& Surface::get_projection(){
    return projection;
}
void Surface::transform(const geo::fCVec3& T, const fMat33 &R){
    V0 = R * V0 + T;
    V1 = R * V1 + T;
    V2 = R * V2 + T;
    V3 = R * V3 + T;

    reset_variables();

}
void Surface::set_normal(const fCVec3 &normal){
    n = normal;
    n = arma::normalise(n);
}
void Surface::compute_projection_on_plane_segement(){
    w = projected_P - V0;
    s = arma::dot(w,arma::cross(n,v)/arma::dot(u,arma::cross(n,v)));
    t = arma::dot(w,arma::cross(n,u)/arma::dot(v,arma::cross(n,u)));
    if(t < 0.0 ){
        t = 0;
    }
    if(t > 1.0){
        t = 1;
    }
    if(s < 0.0){
        s = 0;
    }
    if(s > 1.0){
        s = 1;
    }

    projection = u * s +  v * t + V0;
}
const geo::fCVec3 &Surface::get_middle() const{
    return VMiddle;
}
const fCVec3 &Surface::get_normal() const{
    return n;
}

bool Surface::is_inside(const fCVec3 &P){
    w = P - V0;
    w = arma::normalise(w);
    if(arma::dot(w,n) < 0){
        isInside = true;
    }else{
        isInside = false;
    }
    return isInside;
}

bool Surface::is_inside() const{
    return isInside;
}

void Surface::print() const {

    std::cout<< "geo::Surface info" << std::endl;
    VMiddle.print("VMiddle");
    V0.print("V0");
    V1.print("V1");
    V2.print("V2");
    V3.print("V3");

    w.print("w");
    u.print("u");
    v.print("v");
    projection.print("projection");
    std::cout<< "t: " << t << std::endl;
    std::cout<< "s: " << s << std::endl;


}


Edge::Edge(){
    P1.zeros();
    P2.zeros();
}

Edge::Edge(const Corner& C1,const Corner& C2){
    P1 = C1.C;
    P2 = C2.C;
}

Edge::Edge(const fCVec3& P1, const fCVec3& P2):
    P1(P1),
    P2(P2)
{

}

void Edge::set(const fCVec3& P1, const fCVec3& P2){
    this->P1 = P1;
    this->P2 = P2;
}

double  Edge::shortest_distance(const fCVec3& P)  {
    return distance_point_line_segment(P1,P2,P);
}

const geo::fCVec3& Edge::get_projection(const geo::fCVec3& P){
    distance_point_line_segment(P1,P2,P);
    return projection;
}

geo::fCVec3& Edge::get_projection(){
    return projection;
}


void Edge::transform(const geo::fCVec3& T, const fMat33 &R){
    P1 = R*P1 + T;
    P2 = R*P2 + T;
}

void Edge::print() const{
    std::cout<< P1(0) << "\t" << P2(0)  << std::endl;
    std::cout<< P1(1) << "\t" << P2(1)  << std::endl;
    std::cout<< P1(2) << "\t" << P2(2)  << std::endl;
    std::cout<< std::endl;

}

Corner::Corner(){
    C.zeros();
}

Corner::Corner(const fCVec3& C):C(C){
}

void Corner::set(const fCVec3& C){
    this->C = C;
}

double Corner::shortest_distance(const fCVec3& P){
    projection = C;

    return arma::norm(P - C,2);
}

void Corner::transform(const fCVec3 &T, const fMat33 R){
    C = R * C + T;
}

void Corner::print(const std::string& num) const {
    C.print("C(" + num + ")");
}


}
