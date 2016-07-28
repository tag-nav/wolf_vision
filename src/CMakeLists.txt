
#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#CMAKE modules
SET(CMAKE_MODULE_PATH ${CMAKE_ROOT}/Modules/; ../cmake_modules)
MESSAGE(STATUS ${CMAKE_MODULE_PATH})


#find dependencies. 
FIND_PACKAGE(Ceres QUIET) #Ceres is not required
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: Ceres related sources will be built.")
ENDIF(Ceres_FOUND)

FIND_PACKAGE(faramotics QUIET) #faramotics is not required
IF(faramotics_FOUND)
	FIND_PACKAGE(GLUT REQUIRED)
	FIND_PACKAGE(pose_state_time REQUIRED)
    MESSAGE("Faramotics Library FOUND: Faramotics related sources will be built.")
ENDIF(faramotics_FOUND)

FIND_PACKAGE(laser_scan_utils QUIET) #laser_scan_utils is not required
IF(laser_scan_utils_FOUND)
    MESSAGE("laser_scan_utils Library FOUND: laser_scan_utils related sources will be built.")
ENDIF(laser_scan_utils_FOUND)

FIND_PACKAGE(raw_gps_utils ) #raw_gps_utils is not required
IF(raw_gps_utils_FOUND)
    MESSAGE("raw_gps_utils Library FOUND: raw_gps_utils related sources will be built.")
ENDIF(raw_gps_utils_FOUND)

# OpenCV
FIND_PACKAGE(OpenCV 2 QUIET)
IF(OpenCV_FOUND)
    MESSAGE("OpenCV Library FOUND: OpenCV related sources will be built.")
ENDIF(OpenCV_FOUND)

# YAML with yaml-cpp
#SET(YAMLCPP_USE_STATIC_LIBS TRUE) #Use static library : not working: TO BE REVISED
FIND_PACKAGE(YamlCpp)
IF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library FOUND: yaml-cpp related sources will be built.")
    MESSAGE(STATUS ${YAMLCPP_INCLUDE_DIR})
    MESSAGE(STATUS ${YAMLCPP_LIBRARY})
ELSEIF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library NOT FOUND!")
ENDIF(YAMLCPP_FOUND)
    
# SuiteSparse doesn't have find*.cmake:
FIND_PATH(
    Suitesparse_INCLUDE_DIRS
    NAMES SuiteSparse_config.h
    PATHS /usr/include/suitesparse /usr/local/include/suitesparse)
MESSAGE("Found suitesparse_INCLUDE_DIRS:" ${Suitesparse_INCLUDE_DIRS})

IF(Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND TRUE)
   MESSAGE("Suitesparse FOUND: wolf_solver will be built.")
ELSE (Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND FALSE)
   MESSAGE("Suitesparse NOT FOUND: wolf_solver won't be built.")
ENDIF (Suitesparse_INCLUDE_DIRS)


#include directories
INCLUDE_DIRECTORIES(.)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

IF(faramotics_FOUND)
    INCLUDE_DIRECTORIES(${faramotics_INCLUDE_DIRS})
ENDIF(faramotics_FOUND)

IF(laser_scan_utils_FOUND)
    INCLUDE_DIRECTORIES(${laser_scan_utils_INCLUDE_DIRS})
ENDIF(laser_scan_utils_FOUND)

IF(raw_gps_utils_FOUND)
    INCLUDE_DIRECTORIES(${raw_gps_utils_INCLUDE_DIRS})
ENDIF(raw_gps_utils_FOUND)

# OpenCV
IF(OpenCV_FOUND)
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(OpenCV_FOUND)

IF(Suitesparse_FOUND)
    INCLUDE_DIRECTORIES(${Suitesparse_INCLUDE_DIRS})
ENDIF(Suitesparse_FOUND)

IF(YAMLCPP_FOUND)
    INCLUDE_DIRECTORIES(${YAMLCPP_INCLUDE_DIR})
ENDIF(YAMLCPP_FOUND)

#headers
SET(HDRS_BASE
    wolf.h)

SET(HDRS
    capture_base.h
    capture_fix.h
    capture_gps_fix.h
    capture_motion.h
    capture_motion2.h
    capture_odom_2D.h
    capture_void.h
    constraint_analytic.h
    constraint_base.h
    constraint_container.h
    constraint_corner_2D.h
    constraint_epipolar.h
    constraint_image.h
    constraint_image_new_landmark.h
    constraint_sparse.h
    constraint_fix.h
    constraint_gps_2D.h
    constraint_gps_pseudorange_3D.h
    constraint_gps_pseudorange_2D.h
    constraint_odom_2D.h
    constraint_odom_2D_analytic.h
    constraint_point_2D.h
    constraint_point_to_line_2D.h
    constraint_relative_2D_analytic.h
    constraint_sparse.h
    feature_base.h
    feature_corner_2D.h
    feature_gps_fix.h
    feature_gps_pseudorange.h
    feature_fix.h
    feature_odom_2D.h
    feature_polyline_2D.h
    frame_base.h
    hardware_base.h
    landmark_base.h
    landmark_corner_2D.h
    landmark_container.h
    landmark_point_3d.h
    landmark_line_2D.h
    landmark_polyline_2D.h
    local_parametrization_base.h
    local_parametrization_quaternion.h
    local_parametrization_homogeneous.h
    local_parametrization_polyline_extreme.h
    map_base.h
    motion_buffer.h
    node_base.h
    node_constrained.h
    node_linked.h
    node_terminus.h
    problem.h
    processor_base.h
    processor_imu.h
    processor_factory.h
    processor_motion.h
    processor_odom_2D.h
    processor_odom_3D.h
    processor_tracker.h
    processor_tracker_feature.h
    processor_tracker_feature_dummy.h
    processor_tracker_landmark.h
    processor_tracker_landmark_dummy.h
    sensor_base.h
    sensor_camera.h
    sensor_factory.h
    sensor_gps.h
    sensor_gps_fix.h
    sensor_imu.h
    sensor_odom_2D.h
    state_block.h
    state_homogeneous_3D.h
    state_quaternion.h
    time_stamp.h
    trajectory_base.h
    wolf_manager.h)
   
SET(HDRS_DTASSC
    data_association/matrix.h
    data_association/association_solver.h
    data_association/association_node.h
    data_association/association_tree.h
    data_association/association_nnls.h)
    
#sources
SET(SRCS
    capture_base.cpp
    capture_gps_fix.cpp
    capture_fix.cpp
    capture_motion.cpp
    capture_odom_2D.cpp
    capture_void.cpp
    constraint_base.cpp
    constraint_analytic.cpp
    feature_base.cpp
    feature_corner_2D.cpp
    feature_gps_fix.cpp
    feature_gps_pseudorange.cpp
    feature_fix.cpp
    feature_odom_2D.cpp
    feature_polyline_2D.cpp
    frame_base.cpp
    hardware_base.cpp
    landmark_base.cpp
    landmark_corner_2D.cpp
    landmark_container.cpp
    landmark_point_3d.cpp
    landmark_line_2D.cpp
    landmark_polyline_2D.cpp
    local_parametrization_base.cpp
    local_parametrization_quaternion.cpp
    local_parametrization_homogeneous.cpp
    local_parametrization_polyline_extreme.cpp
    map_base.cpp
    node_base.cpp
    problem.cpp
    processor_base.cpp
    processor_factory.cpp
    processor_imu.cpp
    processor_odom_2D.cpp
    processor_odom_3D.cpp
    processor_tracker.cpp
    processor_tracker_feature.cpp
    processor_tracker_feature_dummy.cpp
    processor_tracker_landmark.cpp
    processor_tracker_landmark_dummy.cpp
    sensor_base.cpp
    sensor_camera.cpp
    sensor_factory.cpp
    sensor_gps.cpp
    sensor_gps_fix.cpp
    sensor_imu.cpp
    sensor_odom_2D.cpp
    time_stamp.cpp
    trajectory_base.cpp
    wolf_manager.cpp
    data_association/association_solver.cpp
    data_association/association_node.cpp
    data_association/association_tree.cpp
    data_association/association_nnls.cpp)

#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(HDRS_WRAPPER
        ceres_wrapper/auto_diff_cost_function_wrapper.h
        ceres_wrapper/ceres_manager.h 
        ceres_wrapper/cost_function_wrapper.h
        ceres_wrapper/create_auto_diff_cost_function.h
        ceres_wrapper/create_auto_diff_cost_function_ceres.h
        ceres_wrapper/create_auto_diff_cost_function_wrapper.h
        ceres_wrapper/create_numeric_diff_cost_function.h
        ceres_wrapper/create_numeric_diff_cost_function_ceres.h
        ceres_wrapper/local_parametrization_wrapper.h )
    SET(SRCS_WRAPPER
        ceres_wrapper/ceres_manager.cpp
        ceres_wrapper/create_auto_diff_cost_function.cpp
        ceres_wrapper/create_numeric_diff_cost_function.cpp
        ceres_wrapper/local_parametrization_wrapper.cpp )
ELSE(Ceres_FOUND)
    SET(HDRS_WRAPPER)
    SET(SRCS_WRAPPER)
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    SET(HDRS ${HDRS}
        capture_laser_2D.h
        sensor_laser_2D.h
        #processor_laser_2D.h
        processor_tracker_landmark_corner.h
        processor_tracker_landmark_polyline.h
        processor_tracker_feature_corner.h
        )
    SET(SRCS ${SRCS}
        capture_laser_2D.cpp
        sensor_laser_2D.cpp
        #processor_laser_2D.cpp
        processor_tracker_landmark_corner.cpp
        processor_tracker_landmark_polyline.cpp
        processor_tracker_feature_corner.cpp
        )
ENDIF(laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    SET(HDRS ${HDRS}
        capture_gps.h
        processor_gps.h
        )
    SET(SRCS ${SRCS}
        capture_gps.cpp
        processor_gps.cpp
        )
ENDIF(raw_gps_utils_FOUND)

# OpenCV 
IF (OpenCV_FOUND)
    SET(HDRS ${HDRS}
        capture_image.h
        feature_point_image.h
        landmark_AHP.h
        processor_image.h
        processor_image_landmark.h
        active_search.h
        )
    SET(SRCS ${SRCS}
        capture_image.cpp
        feature_point_image.cpp
        landmark_AHP.cpp
        processor_image.cpp
        processor_image_landmark.cpp
        active_search.cpp
        )
ENDIF(OpenCV_FOUND)

IF (Suitesparse_FOUND)
    ADD_SUBDIRECTORY(solver)
ENDIF(Suitesparse_FOUND)

# LEAVE YAML FILES ALWAYS IN THE LAST POSITION !!
IF(YAMLCPP_FOUND)
    # headers
    SET(HDRS ${HDRS}
        yaml/yaml_conversion.h
        )

    # sources
    SET(SRCS ${SRCS}
        yaml/sensor_camera_yaml.cpp
        yaml/processor_image_yaml.cpp
        )
    IF(laser_scan_utils_FOUND)
        SET(SRCS ${SRCS}
            yaml/sensor_laser_2D_yaml.cpp
            )
    ENDIF(laser_scan_utils_FOUND)
ENDIF(YAMLCPP_FOUND)



# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS} ${SRCS_WRAPPER})

#Link the created libraries
#=============================================================
IF (Ceres_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${laser_scan_utils_LIBRARY})
ENDIF (laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${raw_gps_utils_LIBRARY})
ENDIF (raw_gps_utils_FOUND)

# OpenCV
IF (OpenCV_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
ENDIF (OpenCV_FOUND)

IF (YAMLCPP_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YAMLCPP_LIBRARY})
ENDIF (YAMLCPP_FOUND)


#install library
#=============================================================
INSTALL(TARGETS ${PROJECT_NAME}
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)
        
#install headers         
INSTALL(FILES ${HDRS_BASE} 
    DESTINATION include/iri-algorithms/wolf)
INSTALL(FILES ${HDRS} 
    DESTINATION include/iri-algorithms/wolf)
INSTALL(FILES ${HDRS_DTASSC} 
    DESTINATION include/iri-algorithms/wolf/data_association)
INSTALL(FILES ${HDRS_WRAPPER} 
    DESTINATION include/iri-algorithms/wolf/ceres_wrapper)
INSTALL(FILES ${HDRS_SOLVER} 
    DESTINATION include/iri-algorithms/wolf/solver)

INSTALL(FILES ../Findwolf.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

#Build examples & tests
MESSAGE("Building examples and tests.")
ADD_SUBDIRECTORY(examples)
