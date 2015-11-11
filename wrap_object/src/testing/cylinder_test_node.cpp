#include <iostream>

#include "visualise/vis_cylinder.h"
#include "visualise/vis_points.h"
#include <ros/ros.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <optitrack_rviz/type_conversion.h>

#include "geometry.h"


int main(int argc,char** argv)
{

    ros::init(argc, argv, "cylinder_test");
    ros::NodeHandle nh;

    /// Create Cylinder
    geo::fCVec3 position;
    position.zeros();
    position(0) = 0.5;
    geo::fMat33 orientation;
    float       radius = 0.1;
    float       length = 1;
    tf::Matrix3x3 rot;
    rot.setRPY(0,0,0);
    opti_rviz::type_conv::tf2mat(rot,orientation);


    geo::Cylinder cylinder("my_cylinder", position,orientation,radius,length);

    opti_rviz::Vis_cylinder vis_cylinder(nh,"cylinder_test");
    vis_cylinder.initialise("world_frame");

    tf::Vector3     position_tf;
    tf::Quaternion  orientation_tf;
    tf::Matrix3x3   orientation33_tf;
    position_tf.setX( position(0) );
    position_tf.setY( position(1) );
    position_tf.setZ( position(2) );
    opti_rviz::type_conv::mat2tf(orientation,orientation33_tf);
    orientation33_tf.getRotation(orientation_tf);
    vis_cylinder.update(position_tf,orientation_tf);

    // create a set of test points

    std::vector<geo::fCVec3> test_points(1);
    std::vector<geo::fCVec3> projections(1);
    std::vector<tf::Vector3> points(1+1);

    test_points[0](0) = 0.1;
    test_points[0](1) = 0;
    test_points[0](2) = 0.4;

  /*  test_points[1](0) =  0;
    test_points[1](1) = -1;
    test_points[1](2) =  0;

    test_points[2](0) =  0.01;
    test_points[2](1) =  0.01;
    test_points[2](2) =  0.01;

    test_points[3](0) =  0;
    test_points[3](1) =  0.1;
    test_points[3](2) =  0;*/

    std::size_t index = 0;
    for(std::size_t i = 0; i < test_points.size();i++){
        test_points[i].print("test_point[" + boost::lexical_cast<std::string>(i) + "]");
        points[index] = tf::Vector3(test_points[i](0),test_points[i](1),test_points[i](2));
        index = index + 1;
    }

    geo::fCVec3 proj;
    for(std::size_t i = 0; i < test_points.size();i++){
        cylinder.projection(test_points[i]);
        projections[i].zeros();
        proj =  cylinder.p_proj;
        //proj.print("projection");
        projections[i] = proj;
        projections[i].print("test_point[" + boost::lexical_cast<std::string>(i) + "]");

        points[index] = tf::Vector3(projections[i](0),projections[i](1),projections[i](2));
        index = index + 1;
    }



    // ---------- Print test Point and Projection

    opti_rviz::Vis_points vis_point(nh,"point_test");
    vis_point.scale = 0.01;


    vis_point.initialise("world_frame",points);
    vis_point.update(points);


    ros::Rate rate(50);
    while(nh.ok()){

        vis_cylinder.publish();
        vis_point.publish();


        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
