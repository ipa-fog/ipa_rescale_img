cmake_minimum_required(VERSION 2.8.3)
project(ipa_rescale_img)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS   
            cv_bridge
            roscpp
            sensor_msgs
            OpenCV
            )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(ipa_rescale_img_node
    src/ipa_rescale_img_node.cpp
    include/ipa_rescale_img_node.h )

add_dependencies(ipa_rescale_img_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(ipa_rescale_img_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ipa_rescale_img_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
