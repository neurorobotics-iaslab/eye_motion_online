cmake_minimum_required(VERSION 3.0)
project(eye_motion_online)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)


find_package(catkin REQUIRED COMPONENTS 
             roscpp 
             std_msgs
             rosneuro_msgs
             message_generation
             eye_decoder
             feedback_cvsa
             geometry_msgs
             )

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    rosneuro_msgs
    message_runtime
    eye_decoder
    geometry_msgs
  DEPENDS
)


###########
## Build ##
###########

include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    )

add_library(${PROJECT_NAME}    
            src/Online_eye.cpp
            )

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      )

add_dependencies(${PROJECT_NAME} 
                 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(online_eye_node src/online_eye_node.cpp)
target_link_libraries(online_eye_node  ${PROJECT_NAME}
                                    ${catkin_LIBRARIES}
                                    )
