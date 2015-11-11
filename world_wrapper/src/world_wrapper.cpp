#include "world_wrapper/world_wrapper.h"
//include "world_wrapper/mesh_loader.h"
//include "world_wrapper/load_mesh.h"

#include <memory>
#include <boost/filesystem.hpp>

#include <iostream>
#include<armadillo>

namespace ww{

World_wrapper::World_wrapper(){
    positions.clear();
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    path_to_save = full_path.string();
    path_to_save = path_to_save + "/saved";
}

void World_wrapper::loadURDF(const std::string& path){
    std::cout<< "world_wrapper:: load urdf " << std::endl;
    model_interface = urdf::parseURDFFile(path);
    std::map<std::string, boost::shared_ptr<urdf::Link> >& Links = model_interface->links_;
    geo::fCVec3 dim, origin,orientation;
    dim.zeros();
    origin.zeros();
    orientation.zeros();


   for( auto it = Links.begin(); it != Links.end();it++ ){
        std::vector<boost::shared_ptr<urdf::Visual> >& visual_array = it->second->visual_array;
        if(visual_array.size() != 0){
            for(std::size_t i = 0; i < visual_array.size();i++){
                boost::shared_ptr<urdf::Visual>& visual =  visual_array[i];
                if(visual->geometry != NULL){
                    boost::shared_ptr<urdf::Geometry>& geometry = visual->geometry;
                    switch (geometry->type) {
                    case urdf::Geometry::BOX:
                     {
                        urdf::Box* box = dynamic_cast<urdf::Box*>(geometry.get());
                        dim[0] = box->dim.x; dim[1] = box->dim.y; dim[2] = box->dim.z;
                        std::string name = it->first;
                        std::cout<< "\t" << name << ": (" << dim[0] << "," << dim[1] << "," << dim[2] << ")" << std::endl;

                        //wrapped_objects.push_back_box(name,dim,origin,orientation);
                        bInitialised[name] = false;
                        break;
                     }
                    case urdf::Geometry::MESH:
                    {
                       break;
                    }
                    default:
                        break;
                    }



                }
            }
        }

   }

}

void World_wrapper::load_circle(const std::string& name,const std::string &path){
    arma::fmat param;
    ROS_INFO("load CIRCLE");
    param.load(path,arma::csv_ascii);
    std::cout<< param.n_rows << "\t" << param.n_cols << std::endl;
    geo::fCVec3 C = {{param(0,0),param(0,1),param(0,2)}}; /// position
    geo::fCVec3 R = {{param(0,3),param(0,4),param(0,5)}}; /// row, yaw, pitch
    float       r = param(0,6);
    wrapped_objects.push_back_circle(name,C,R,r);
}

void World_wrapper::load_mesh(const std::string& path){
  //  Load_mesh load_mesh;
  //  load_mesh.load_mesh_ogre(path);
}

std::string World_wrapper::get_link_name(std::size_t i){
   return wrapped_objects.get_wobject_name(i);
}

int World_wrapper::get_link_id(const std::string& name){
    return wrapped_objects.get_wobject_index(name);
}

void World_wrapper::set_origin_orientation(const std::string& name,
                                          const geo::fCVec3 &origin,
                                          const geo::fCVec3 &orientation){

    wrapped_objects.set_origin_orientation(name,origin,orientation);
    bInitialised[name] = true;
}

std::size_t World_wrapper::get_number_objects(){
    return wrapped_objects.get_number_objects();
}

wobj::WBox &World_wrapper::get_wbox(std::size_t index){
    return wrapped_objects.get_wbox(index);
}

wobj::WBox& World_wrapper::get_wbox(const std::string& name){
    int index = wrapped_objects.get_wobject_index(name);
    return wrapped_objects.get_wbox(index);
}

bool World_wrapper::isInitialised(){
    for(auto it = bInitialised.begin(); it != bInitialised.end();it++){
            if( !(it->second) ){
                  return false;
            }
    }
    return true;
}

void World_wrapper::print_points(const std::string& type){
    wrapped_objects.print_points(type);
}

void World_wrapper::initialise_origin_orientation(ww::World_wrapper& world_wrapper, const std::string& root_link_name, bool re_initialise){

   /* if(re_initialise){
        for(auto it = bInitialised.begin(); it != bInitialised.end();it++){
            it->second = false;
        }
    }*/



    ros::Rate rate(1.0);
    tf::TransformListener listener;

    while (!world_wrapper.isInitialised()){

        ROS_INFO("initialise world_wrapper");
        std::cout<< "num obj2 : " << wrapped_objects.get_number_objects() << std::endl;
        std::cout<< "num obj:   " << world_wrapper.get_number_objects() << std::endl;

        tf::StampedTransform transform;
        tf::Vector3 origin, orientation;
        geo::fCVec3 origin_,orientation_;
        std::string link_name;
        std::string traget_frame, source_frame;

        try{

            for(std::size_t i = 0; i < 5;i++){
                link_name = world_wrapper.get_link_name(i);


                std::cout<< "object("<<i<<"): " << link_name << std::endl;


                traget_frame = "/" + root_link_name;
                source_frame =  "/" + link_name;

                listener.lookupTransform(traget_frame,source_frame, ros::Time(0), transform);

                ROS_INFO("target_frame : %s   source_frame : %s ",traget_frame.c_str(),source_frame.c_str());

                origin      = transform.getOrigin();
                orientation = transform.getRotation().getAxis();
                origin_(0) = origin[0];
                origin_(1) = origin[1];
                origin_(2) = origin[2];// - 0.02;

                orientation_[0] = transform.getRotation().getX();
                orientation_[1] = transform.getRotation().getY();
                orientation_[2] = transform.getRotation().getZ();

                world_wrapper.set_origin_orientation(link_name,origin_,orientation_);
                positions[link_name] = std::array<float,6>{{origin_[0],origin_[1],origin_[2],orientation_[0],orientation_[1],orientation_[2]}};

                ROS_INFO("transformed %s",link_name.c_str());
            }

        }catch (tf::TransformException ex){

            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();

        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("all objects origin and orientation have been set");
}

bool World_wrapper::save_transform(const std::string& folder_name){

    std::string folder_save = path_to_save + "/" + folder_name;
    if( !(boost::filesystem::exists(folder_save) && boost::filesystem::is_directory(folder_save) )){
        if (boost::filesystem::create_directory(folder_save)){
            std::cout << "new folder " + folder_name + " created" << "\n";
        }else{
            std::cerr << "FAILED to create new folder " + folder_name << "\n";
            return false;
        }
    }

    // save all the urdf transformation
    for(auto it = positions.begin(); it != positions.end(); it++){
        std::string file_name = folder_save + "/" + it->first + ".csv";
        std::ofstream file_stream(file_name);
        if(file_stream.is_open()){
            std::array<float,6>& p = it->second;
            file_stream << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << ","
                        << p[4] << "," << p[5] << std::endl;
            file_stream.close();
        }else{
            std::cerr << " FAILED to open: " + file_name << std::endl;
            return false;
        }
    }


    return true;
}

bool World_wrapper::load_transform(const std::string& folder_name){

    std::string folder_load = path_to_save + "/" + folder_name;
    if( !(boost::filesystem::exists(folder_load) && boost::filesystem::is_directory(folder_load))){
        if (boost::filesystem::create_directory(folder_load)){
            std::cout << "new folder " + folder_name + " created" << "\n";
        }else{
            std::cerr << "FAILED to create new folder " + folder_name << "\n";
            return false;
        }
    }

    boost::filesystem::directory_iterator end_iter;
    std::string link_name,file_name;
    std::array<float,6> position;
    std::array<float,3> origin, orientation;

    for(boost::filesystem::directory_iterator dir_iter(folder_load) ; dir_iter != end_iter ; ++dir_iter)
     {
       if (boost::filesystem::is_regular_file(dir_iter->status()) )
       {
           boost::filesystem::path p(dir_iter->path());
           file_name = p.string();
           link_name = p.stem().string();



            std::ifstream file_stream(file_name);
             if(!file_stream)
             {
               std::cerr << "Could not open file: " << file_name << std::endl;
               return false;
             }

             std::string line;
             std::vector< std::string > vec;
             while( getline(file_stream, line) ){



             }
/*
             origin[0] = position[0];
             origin[1] = position[1];
             origin[2] = position[2];

             orientation[0] = position[3];
             orientation[1] = position[4];
             orientation[2] = position[5];
             std::cout<< "load ==> " << link_name << std::endl;
             std::cout<< "origin:\t" << "(" << origin[0] << "," << origin[1] << "," << origin[2] << ")" << std::endl;
             std::cout<< "orientation:\t" << "(" << orientation[0] << "," << orientation[1] << "," << orientation[2] << ")" << std::endl;

*/
               // world_wrapper.set_origin_orientation(link_name,origin_,orientation_);*/


       }
     }

   return true;
}


}
