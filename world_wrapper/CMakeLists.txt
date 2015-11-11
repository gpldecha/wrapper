cmake_minimum_required(VERSION 2.8.3)
project(world_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS urdf roscpp tf visualization_msgs resource_retriever rviz objects optitrack_rviz wrap_object)

add_service_files(
  FILES
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#set(OGRE_PATH /usr/lib/x86_64-linux-gnu/OGRE-1.8.0)

find_package(Boost REQUIRED)
find_package(Armadillo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV)

find_package(PkgConfig)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
    pkg_check_modules(ASSIMP assimp)
endif()

if (ASSIMP_FOUND)
    if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
        set(IS_ASSIMP3 1)
        add_definitions(-DIS_ASSIMP3)
        message(STATUS "Found assimp v3")
    else()
        message(STATUS "Found assimp v2")
    endif()

    include_directories(${ASSIMP_INCLUDE_DIRS})
    link_directories(${ASSIMP_LIBRARY_DIRS})
else()
    message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
    set(ASSIMP_LIBRARIES assimp)
    set(ASSIMP_LIBRARY_DIRS)
    set(ASSIMP_CXX_FLAGS)
    set(ASSIMP_CFLAGS_OTHER)
    set(ASSIMP_LINK_FLAGS)
    set(ASSIMP_INCLUDE_DIRS)
    set(IS_ASSIMP3 0) # most likely not
endif()


catkin_package(
   INCLUDE_DIRS include
   DEPENDS Boost
   CATKIN_DEPENDS urdf roscpp tf message_runtime resource_retriever rviz
   LIBRARIES
   ${PROJECT_NAME}
   wrap_object	
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-unused-variable")        ## Optimize
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)



include_directories(${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIR}
                    ${INCLUDE_DIR}/world_wrapper
)

set(W_HEADER_FILES ${INCLUDE_DIR}/world_wrapper/world_wrapper.h
                   ${INCLUDE_DIR}/node/service.h
                   ${INCLUDE_DIR}/node/publisher.h
)

set(W_SRC_FILES ${SRC_DIR}/world_wrapper.cpp
                ${SRC_DIR}/service.cpp
                ${SRC_DIR}/publisher.cpp
)

#set(VIS_HEADER_FILES ${INCLUDE_DIR}/world_wrapper/visualisation/vis_wbox.h
#                     ${INCLUDE_DIR}/world_wrapper/visualisation/vis_features.h
#)

#set(VIS_SRC_FILES ${SRC_DIR}/visualisation/vis_wbox.cpp
#                  ${SRC_DIR}/visualisation/vis_features.cpp
#)


#add_library(${PROJECT_NAME} SHARED  ${W_SRC_FILES} ${W_HEADER_FILES} ${VIS_HEADER_FILES} ${VIS_SRC_FILES} )
add_library(${PROJECT_NAME} SHARED  ${W_SRC_FILES} ${W_HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} armadillo)

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} armadillo libOgreMain.so assimp)
#add_executable(${PROJECT_NAME}_node  ${SRC_DIR}/run.cpp)
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(test_world_wrapper ${SRC_DIR}/test.cpp   )
#target_link_libraries(test_world_wrapper ${PROJECT_NAME})

