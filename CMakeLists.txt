cmake_minimum_required(VERSION 3.0.2)
project(phoxi_trigger)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        pcl_conversions
        std_msgs
)
find_package(PCL REQUIRED)


set(CMAKE_CXX_STANDARD 20)

catkin_package()

if (NOT PHO_API_CMAKE_CONFIG_PATH)
    set(PHO_API_CMAKE_CONFIG_PATH "$ENV{PHOXI_CONTROL_PATH}")
endif ()

find_package(PhoXi REQUIRED CONFIG PATHS "${PHO_API_CMAKE_CONFIG_PATH}")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(TriggerCameras src/phoxi_trigger_node.cpp)
#add_executable(TriggerCameras src/MinimalPclExample.cpp)
target_link_libraries(TriggerCameras ${catkin_LIBRARIES})
target_link_libraries(TriggerCameras ${PHOXI_LIBRARY} $<$<PLATFORM_ID:Linux>:rt> ${PCL_LIBRARIES})
target_include_directories(TriggerCameras PUBLIC ${PHOXI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
