
#set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 2.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common io visualization)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4" )
add_definitions( ${PCL_DEFINITIONS} )
include_directories( ${PCL_INCLUDE_DIRS} )


#link_libraries( ${PCL_LIBRARY_DIRS} )
#include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} )


#add_executable( generate_pointcloud PointCloud.cpp )
#target_link_libraries( generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_library(slambase slamBase.cpp )
target_link_libraries(slambase ${OpenCV_LIBS} ${PCL_LIBRARIES} )


add_executable(joinPointCloud joinPointCloud.cpp)
target_link_libraries(joinPointCloud slambase ${OpenCV_LIBS} ${PCL_LIBRARIES} ${EIGEN_LIBRARIES} )
#target_link_libraries(joinPointCloud slambase -nostartfiles )
