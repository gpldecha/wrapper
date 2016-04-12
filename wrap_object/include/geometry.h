#ifndef GEOMETRY_H_
#define GEOMETRY_H_

// armadillo
#include<armadillo>
#include <array>

namespace geo{

typedef arma::fcolvec3 fCVec3;
typedef arma::fcolvec4 fCVec4;
typedef arma::fmat33   fMat33;
typedef arma::fmat44   fMat44;


class Corner;


class Circle{

public:

    Circle();

    Circle(const std::string& name, const geo::fCVec3 &C, const geo::fMat33& R, float r);

    geo::fCVec3 &get_projection();

    void transform(const geo::fCVec3& t, const geo::fMat33& Rot);

    float shortest_distance(const geo::fCVec3& P_w);

    void print() const;

public:

    fCVec3        K;     /// Projection
    fCVec3        C;     /// Center of circle
    fMat33        R;     /// rotation matrix
    float         r;     /// radius of circle
    fCVec3        N,U,V; /// normal of the circle
    std::string   name;
    fMat33        invR;
    fCVec3        invC;


protected:



  fCVec3        P_c;   /// input point in the frame of reference of the circle
  fCVec4        P_tmp;


};


class Disk : public Circle{

public:

    Disk();

    Disk(const std::string& name, const geo::fCVec3& C, const geo::fMat33& R, float r);

    geo::fCVec3 &get_projection();

    float shortest_distance(const geo::fCVec3& P_w);

public:

    bool    bIn; /// if the projection is inside the disk or not;
    float   dist_edge;
    float   dist_surface;
    fCVec3  Q;  /// projection of P_w onto the plane of the circle

};

class Cylinder{

public:

    Cylinder();

    Cylinder(const std::string name, const geo::fCVec3& x_ref, const geo::fMat33& R, float radius,float length);

    void projection(geo::fCVec3& P);

    bool is_inside(geo::fCVec3& P);

public:

    std::string name;
    geo::fCVec3 x_ref;
    geo::fMat33 R, RT;
    geo::fCVec3 p_proj;
    arma::fvec2 tmp;
    geo::fCVec3 X_c;
    float radius;
    float length;
};



class Geometry{

public:

	virtual ~Geometry();

    virtual double shortest_distance(const geo::fCVec3& P) = 0;

    double distance_point_line_segment(const geo::fCVec3& v,const geo::fCVec3& w, const geo::fCVec3& p);

    std::array<float,3> &GetProjection();

    geo::fCVec3 &get_projection();

    bool isBigger(const geo::fCVec3& v1,const geo::fCVec3& v2);

private:
    float l;
    float distance;
    float t;
    geo::fCVec3 v,w;
protected:
    geo::fCVec3 projection;
    std::array<float,3> proj;
};

class Surface : public Geometry{

public:

    Surface();

    Surface(const Corner& C0,const Corner& C1,const Corner& C2,const Corner& C3);

    virtual double  shortest_distance(const geo::fCVec3& P);

    const geo::fCVec3& get_projection(const geo::fCVec3& P);

    geo::fCVec3& get_projection();

    void transform(const geo::fCVec3& T,const geo::fMat33& R);

    void set_normal(const geo::fCVec3& normal);

    const geo::fCVec3 &get_middle() const;

    const geo::fCVec3 &get_normal() const;

    void print() const;

    bool is_inside(const geo::fCVec3& P);

    bool is_inside() const;

private:

    void init();

    void reset_variables();

    void compute_projection_on_plane_segement();

private:

    geo::fCVec3 V0,V1,V2,V3; // points define a plane;
    geo::fCVec3 VMiddle;
    geo::fCVec3 u,v,w; //vectors from VO1 and VO2
    geo::fCVec3 n; // normal to the plane
    double d;
    geo::fCVec3 projected_P;
    std::vector<double> dists;
    double s,t;
    bool isInside;

};

class Edge : public Geometry {

public:

	Edge();

    Edge(const Corner& C1,const Corner& C2);

    Edge(const fCVec3& P1, const fCVec3& P2);

    void set(const fCVec3& P1, const fCVec3& P2);

    virtual double  shortest_distance(const fCVec3& P);

    const geo::fCVec3& get_projection(const geo::fCVec3& P);

    geo::fCVec3& get_projection();

    void transform(const geo::fCVec3& T,const geo::fMat33& R);

    void print() const;

public:

    fCVec3 P1,P2;

};

class Corner :public Geometry{
public:

    Corner();

    Corner(const geo::fCVec3& C);

    void set(const geo::fCVec3& C);

    virtual double  shortest_distance(const geo::fCVec3& P);

    void transform(const geo::fCVec3& T,const geo::fMat33 R);

    void print(const std::string &num) const;


public:
    geo::fCVec3 C;

};

}


#endif
