#ifndef SOCKET_ONE_H_
#define SOCKET_ONE_H_

#include "wsocket.h"
#include "wbox.h"
#include <string>

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace obj{

class Socket_one{

public:

    Socket_one(std::string link_socket_name="socket_default",
               std::string link_box_name="link_socket",
               const tf::Vector3& origin=tf::Vector3(0,0,0),
               const tf::Vector3& rpy=tf::Vector3(0,0,0),
               float scale = 1.0);

public:

    wobj::WSocket            wsocket;
    wobj::WBox               wbox;
    std::array<wobj::WBox,3> hole_wboxes;

private:

    float disk_radius;
    float hole_radius;

};

}

#endif
