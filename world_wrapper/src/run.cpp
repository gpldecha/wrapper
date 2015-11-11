// ROS

#include<ros/ros.h>


// STL

#include <string>
#include <vector>


// world_wrapper code

#include "world_wrapper/world_wrapper.h"
#include "node/publisher.h"

#include <objects/socket_one.h>
#include <objects/vis_socket.h>

#include <optitrack_rviz/input.h>




int find_index(int argc, char **argv,std::string str){
    std::string tmp;
    for(int i = 0; i < argc;i++){
        tmp = argv[i];
        if(tmp == str){
            return i;
        }
    }
    return -1;
}

bool process_input(int argc, char **argv,std::vector<std::string>& urdf_files,std::string& root_link_name){


    if(argc < 5){
        ROS_ERROR("options -urdf file1 file2 ...  --root_link root_link_name");
        return false;
    }else{
        int index_urdf,index_root_link;
        index_root_link = find_index(argc,argv,"-root_link");
        index_urdf      = find_index(argc,argv,"-urdf");


        ROS_INFO("index_urdf: %d index_root: %d",index_urdf,index_root_link);

        if(index_urdf == -1){
            ROS_ERROR("option -urdf not specified!");
            return false;
        }
        if(index_root_link == -1){
            ROS_ERROR("option -root_link not specified!");
            return false;
        }

        root_link_name = argv[index_root_link+1];
        int start =  index_urdf+1;

       /* for(std::size_t i = 0; i < argc;i++){
            std::cout<< "argv["<<i<<"]: " << argv[i] << std::endl;
        }*/

        std::string file;
        for(int i = start; i < argc;i++){
            file = argv[i];
            if(file.substr(file.find_last_of(".") + 1) == "urdf"){
                urdf_files.push_back(argv[i]);
            }
        }

        if(urdf_files.size() == 0){
            ROS_ERROR("No urdf files were found, you should specify them after option -urdf [full_path_to_urdf_file]!");
        }



        return true;
    }

}

int main(int argc,char **argv){


    std::vector<std::string> urdf_files(0);
    std::string root_link_name;
    ww::World_wrapper world_wrapper;




    if(!process_input(argc,argv,urdf_files,root_link_name)){
        ROS_ERROR(" did not manage to process input URDF files!");
        return -1;
    }

    for(std::size_t i = 0; i < urdf_files.size();i++){
        std::cout<< "urdf_files["<<i<<"]:\t" << urdf_files[i] << std::endl;
    }


    ROS_INFO("root_link = %s",root_link_name.c_str());

    unsigned found;
    for(std::size_t i = 0; i < urdf_files.size(); i++){
        found = urdf_files[i].find_last_of("/\\");
        ROS_INFO("loading ====> %s", urdf_files[i].substr(found+1).c_str());
      //  world_wrapper.loadURDF(urdf_files[i]);
    }

    ROS_INFO("Get transformation of URDF's");

    ros::init(argc, argv, "world_wrapper");
    ros::NodeHandle node;

    geo::fCVec3 origin_      = {{0,0,-0.02/2}};
    geo::fCVec3 dim_         = {{0.8,0.4,0.02}};
    geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

    wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

    /// add a socket
    tf::Vector3 origin(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);

    obj::Socket_one socket_one("socket_one",origin,rpy,1);

    world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);
    world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);



    /// feature publisher

    ww::Publisher publisher("visualization_marker",&node,&world_wrapper);
    publisher.init("/world");

    /// socket publisher

    obj::Vis_socket vis_socket(node,socket_one.wsocket);
    vis_socket.initialise(100,0.001);

    geo::fCVec3 C;
    geo::fCVec3 P;

    publisher.update_position();
    publisher.update_position_test_point(P);




   C(0) = 0;
   C(1) = 0;
   C(2) = 0.02;

    float angle = 0;
    float r_x     = 0.25;
    float r_y     = 0.05;
    float r_z     = 0.002;

    ros::Rate rate(50.0);
    while(node.ok()){


        angle = angle + 0.01 ;
        P(0) = C(0) + r_x * cos(angle);
        P(1) = C(1) + r_y * sin(angle * 5);
        P(2) = C(2) + r_z * cos(angle);

        world_wrapper.wrapped_objects.distance_to_features(P);

        publisher.update_position();
        publisher.update_position_test_point(P);

        world_wrapper.wrapped_objects.get_closest_point_edge().print("edge");
        world_wrapper.wrapped_objects.get_closest_point_surface().print("surf");


        publisher.publish();
        vis_socket.publish();

        ros::spinOnce();
        rate.sleep();
    }


    return 0;

}
