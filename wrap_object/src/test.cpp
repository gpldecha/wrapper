#include "wrapobject.h"


int main(int argc,char** argv){




    wobj::WrapObject wrapobject;

    geo::fCVec3 dim                 = {{1,1,1}};
    geo::fCVec3 origin              = {{0,0,0}};
    geo::fCVec3 orientation         = {{0,0,0}};



    /*wrapobject.push_back_box("my_box",dim,origin,orientation);

    geo::fCVec3 P = {{2,0,0}};
    wrapobject.distance_to_features(P);

    float dist_s = wrapobject.get_distance_to_surface();
    float dist_e = wrapobject.get_distance_to_edge();
    float dist_c = wrapobject.get_distance_to_corner();

    std::cout<< "distances s: " << dist_s << "\t" << dist_e << "\t" << dist_c << std::endl;*/


	return 0;
}
