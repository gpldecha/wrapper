#ifndef WORLD_WRAPPER_H_
#define WORLD_WRAPPER_H_

// Boost

#include <boost/tokenizer.hpp>

// STL

#include <string>
#include <array>
#include <map>
#include <fstream>
#include <sstream>

// ROS

#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// my libraries


#include "wrapobject.h"

namespace ww {


class World_wrapper{

    typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;

public:

    World_wrapper();

    void loadURDF(const std::string &path);

    void load_circle(const std::string &name, const std::string &path);

    void load_mesh(const std::string& path);

    std::string get_link_name(std::size_t i);

    int         get_link_id(const std::string& name);

    std::size_t get_number_objects();

    wobj::WBox &get_wbox(std::size_t index);

    wobj::WBox& get_wbox(const std::string& name);

    bool isInitialised();

    void print_points(const std::string& type);

    void set_origin_orientation(const std::string& name,const geo::fCVec3& origin, const geo::fCVec3& orientation);

    void initialise_origin_orientation(ww::World_wrapper& world_wrapper,const std::string& root_link_name,bool re_initialise=false);

    bool save_transform(const std::string& folder_name);

    ///
    /// \brief load_transform
    /// \param[in] folder_name (relative to word_wrapper/saved/ folder) in which transformation files are located
    /// \return[out] if the operation was sucessfull or not
    ///
    bool load_transform(const std::string& folder_name);

private:

    void inline set_zero(std::array<float,3>& arr){
            arr[0] = 0; arr[1] = 0; arr[2] = 0;
    }

public:

    wobj::WrapObject wrapped_objects;



private:

    std::map<std::string,bool> bInitialised;

    boost::shared_ptr<urdf::ModelInterface> model_interface;

    std::map<std::string,std::array<float,6> > positions;

    std::string path_to_save;


};

}

#endif
