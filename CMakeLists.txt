cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_GTSAM_USE_SYSTEM_EIGEN ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

project(low_cost_underwater_slam)





find_package(ament_cmake REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(commonbluerovmsg REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
#find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
#find_package(OpenGR REQUIRED)
#find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
#find_package(fs2d REQUIRED)
find_package(OpenCV 4.6.0 REQUIRED )
#find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#find_package(PythonLibs REQUIRED)
#find_package(mocap_msgs REQUIRED)
#find_package(gmm_registration REQUIRED)
find_package(px4_msgs REQUIRED)
#find_package(micron_driver_ros REQUIRED)
find_package(waterlinked_a50 REQUIRED)
find_package(usbl_seatrac_msgs REQUIRED)
#find_package(rosbag2_cpp REQUIRED)

#find_package(perception_oru REQUIRED)


################################################################################
# general interface
################################################################################
add_library(generalinterface INTERFACE)
target_include_directories(generalinterface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/generalInterface/include)

################################################################################
# json
################################################################################
#add_library(json SHARED src/json/src/jsoncpp.cpp)
#target_include_directories(json PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/json/include)


################################################################################
# generalhelpfullib
################################################################################
add_library(generalhelpfullib SHARED src/generalHelpfulTools/src/generalHelpfulTools.cpp src/generalHelpfulTools/src/PeakFinder.cpp)
ament_target_dependencies(generalhelpfullib tf2 Eigen3 GTSAM)
target_include_directories(generalhelpfullib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/generalHelpfulTools/include)


################################################################################
# posegraph
################################################################################

add_library(posegraph SHARED src/poseGraph/src/edge.cpp src/poseGraph/src/graphSlamSaveStructure.cpp src/poseGraph/src/vertex.cpp)
ament_target_dependencies(posegraph PUBLIC Eigen3 GTSAM)
target_include_directories(posegraph PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/poseGraph/include)
target_link_libraries(posegraph PUBLIC generalinterface generalhelpfullib gtsam)

################################################################################
# 2D Peak Algorithm
################################################################################

add_library(findpeaks INTERFACE)
target_include_directories(findpeaks INTERFACE
        src/registration/find-peaks/include
)
add_library(UNION_FIND
        SHARED
        src/registration/find-peaks/src/union_find.cpp
)
target_include_directories(UNION_FIND PRIVATE
        src/registration/find-peaks/include
)
target_link_libraries(findpeaks INTERFACE UNION_FIND)



################################################################################
# soft20
################################################################################



add_library(soft20 SHARED

        src/registration/soft20/src/lib1/rotate_so3_fftw.c
        src/registration/soft20/src/lib1/rotate_so3_utils.c
        src/registration/soft20/src/lib1/s2_cospmls.c
        src/registration/soft20/src/lib1/s2_legendreTransforms.c
        src/registration/soft20/src/lib1/s2_semi_fly.c
        src/registration/soft20/src/lib1/s2_semi_memo.c
        src/registration/soft20/src/lib1/so3_correlate_fftw.c
        src/registration/soft20/src/lib1/soft_fftw.c
        src/registration/soft20/src/lib1/soft_fftw_nt.c
        src/registration/soft20/src/lib1/soft_fftw_pc.c
        src/registration/soft20/src/lib1/utils_vec_cx.c
        src/registration/soft20/src/lib1/wignerTransforms_fftw.c
        src/registration/soft20/src/lib1/wrap_s2_rotate_fftw.c
        src/registration/soft20/src/lib1/wrap_soft_fftw.c
        src/registration/soft20/src/lib1/wrap_soft_fftw_cor2.c
        src/registration/soft20/src/common/csecond.c
        src/registration/soft20/src/common/makeweights.c
        src/registration/soft20/src/common/makeWigner.c
        src/registration/soft20/src/common/s2_primitive.c
        src/registration/soft20/src/common/utils_so3.c
        src/registration/soft20/src/common/wignerTransforms.c
)

target_link_libraries(soft20 fftw3)
target_include_directories(soft20 PUBLIC src/registration/soft20/include/soft20 src/registration/soft20/include )

#install(
#        DIRECTORY src/registration/soft20/include/soft20
#        DESTINATION src/registration/soft20/include
#)
#
#install(
#        TARGETS soft20
#        EXPORT soft20
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib
#        RUNTIME DESTINATION bin
#        INCLUDES DESTINATION include
#)


#ament_export_targets(soft20 HAS_LIBRARY_TARGET)

################################################################################
# fs2d
################################################################################

add_library(fs2d SHARED src/registration/fs2D/src/softCorrelationClass.cpp src/registration/fs2D/src/softRegistrationClass.cpp)
ament_target_dependencies(fs2d Eigen3 rclcpp tf2 tf2_geometry_msgs GTSAM )
target_include_directories(fs2d PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/fs2D/include ${OpenCV_INCLUDE_DIRS}) #${PCL_INCLUDE_DIRS}
target_link_libraries(fs2d generalhelpfullib soft20 findpeaks ${OpenCV_LIBS}) # ${PYTHON_LIBRARIES}



################################################################################
# scan Registration
################################################################################

#add_library(registrationlib SHARED src/registration/src/scanRegistrationClass.cpp)
#ament_target_dependencies(registrationlib Eigen3 rclcpp tf2 tf2_geometry_msgs GTSAM )
#target_include_directories(registrationlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/include ${OpenCV_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS} ${INCLUDE_DIRECTORIES_ORU}) #${PCL_INCLUDE_DIRS}
#target_link_libraries(registrationlib generalhelpfullib ${OpenCV_LIBS} ${PCL_LIBRARIES} ) # ${PYTHON_LIBRARIES}


################################################################################
# slamtools
################################################################################

add_library(slamtools SHARED src/slamTools/src/slamToolsRos.cpp)
ament_target_dependencies(slamtools Eigen3 rclcpp visualization_msgs nav_msgs GTSAM)
target_include_directories(slamtools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/slamTools/include)
target_link_libraries(slamtools posegraph fs2d)


################################################################################
############################ Executables #######################################
################################################################################



################################################################################
# EKF
################################################################################
add_executable(ekfNode src/stateEstimation/ekfDVLnode.cpp src/stateEstimation/pose.h src/stateEstimation/ekfDVL.cpp src/stateEstimation/ekfDVL.h  )
ament_target_dependencies(ekfNode rclcpp std_msgs sensor_msgs waterlinked_a50 usbl_seatrac_msgs tf2 Eigen3 px4_msgs)
target_link_libraries(ekfNode generalhelpfullib)

################################################################################
# SLAM
################################################################################
add_executable(rosSlamTest src/slamRuns/rosSlamExp1.cpp )
ament_target_dependencies(rosSlamTest GTSAM geometry_msgs nav_msgs ping360_sonar_msgs )
target_link_libraries(rosSlamTest ${OpenCV_LIBS} posegraph slamtools generalhelpfullib fs2d)
#target_include_directories(rosSlamTest PUBLIC ${INCLUDE_DIRECTORIES_ORU})


################################################################################
# INSTALL LIBRARIES
################################################################################
install(TARGETS
        generalhelpfullib
        slamtools
        fs2d
        posegraph
        soft20
        UNION_FIND
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)
################################################################################
# INSTALL EXECUTABLES
################################################################################
install(TARGETS
        ekfNode
        rosSlamTest

        DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# INSTALL Configs
################################################################################
install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


#install(
#        DIRECTORY lib/
#        DESTINATION lib
#)




ament_package()
