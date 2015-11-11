cmake_minimum_required(VERSION 2.8.3)
project(wrap_object)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp optitrack_rviz)


#   SET FLAGS

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -W -Wno-unused-function -Wno-unused-parameter")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
# FIND LIBRARIES

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
endif()

find_package(Armadillo)

if(ARMADILLO_FOUND)
    message(STATUS "${Green} libarmadillo found ${ColourReset} "            )
    message(STATUS "    ${ARMADILLO_INCLUDE_DIRS}"                          )
    message(STATUS "    ${ARMADILLO_LIBRARIES}"                             )
elseif(ARMADILLO_FOUND)
    message(FATAL_ERROR "${Red} libarmadillo NOT FOUND! ${ColourReset}"     )
endif(ARMADILLO_FOUND)

catkin_package(
   INCLUDE_DIRS include
   DEPENDS Boost
   CATKIN_DEPENDS roscpp
   LIBRARIES
   ${PROJECT_NAME}
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include )
set(SRC_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/src     )

set(HEADER_FILES ${INCLUDE_DIR}/wrapobject.h
                 ${INCLUDE_DIR}/wbox.h
                 ${INCLUDE_DIR}/wsocket.h
                 ${INCLUDE_DIR}/geometry.h
                 ${INCLUDE_DIR}/spacepartition.h

)
set(SRC_FILES    ${SRC_DIR}/wrapobject.cpp
                 ${SRC_DIR}/wbox.cpp
                 ${SRC_DIR}/wsocket.cpp
                 ${SRC_DIR}/geometry.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES}  )
add_executable(test_${PROJECT_NAME} ${SRC_DIR}/test.cpp ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES} ${LIB_INTEL})
target_link_libraries(test_${PROJECT_NAME} ${ARMADILLO_LIBRARIES})

add_executable(test_cylinder_node ${SRC_DIR}/testing/cylinder_test_node.cpp )
target_link_libraries(test_cylinder_node ${PROJECT_NAME} ${catkin_LIBRARIES})




set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH    ${CMAKE_CURRENT_SOURCE_DIR}/lib)

