
#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

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

IF(Suitesparse_FOUND)
    INCLUDE_DIRECTORIES(${Suitesparse_INCLUDE_DIRS})
ENDIF(Suitesparse_FOUND)

#headers
SET(HDRS_BASE
    wolf.h)

SET(HDRS
    capture_base.h
    capture_relative.h
    capture_fix.h
    capture_gps_fix.h
    capture_odom_2D.h
    constraint_base.h
    constraint_sparse.h
    constraint_gps_2D.h
    constraint_fix.h
    constraint_odom_2D_theta.h
    constraint_odom_2D_complex_angle.h
    constraint_corner_2D_theta.h
    constraint_container.h
    feature_base.h
    feature_corner_2D.h
    feature_gps_fix.h
    feature_fix.h
    feature_odom_2D.h
    frame_base.h
    landmark_base.h
    landmark_corner_2D.h
    landmark_container.h
    map_base.h
    node_base.h
    node_terminus.h
    node_linked.h
    sensor_base.h
    sensor_laser_2D.h
    sensor_odom_2D.h
    sensor_gps_fix.h
    state_base.h
    state_point.h
    state_orientation.h
    state_quaternion.h
    state_theta.h
    state_complex_angle.h
    time_stamp.h
    trajectory_base.h
    wolf_problem.h
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
    capture_relative.cpp
    capture_gps_fix.cpp
    capture_fix.cpp
    capture_odom_2D.cpp
    constraint_base.cpp
    feature_base.cpp
    feature_corner_2D.cpp
    feature_gps_fix.cpp
    feature_fix.cpp
    feature_odom_2D.cpp
    frame_base.cpp
    landmark_base.cpp
    landmark_corner_2D.cpp
    landmark_container.cpp
    map_base.cpp
    node_base.cpp
    node_terminus.cpp
    sensor_base.cpp
    sensor_odom_2D.cpp
    sensor_gps_fix.cpp
    state_base.cpp
    state_orientation.cpp
    state_quaternion.cpp
    state_theta.cpp
    state_complex_angle.cpp
    time_stamp.cpp
    trajectory_base.cpp
    wolf_problem.cpp
    data_association/association_solver.cpp
    data_association/association_node.cpp
    data_association/association_tree.cpp
    data_association/association_nnls.cpp)

#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(HDRS_WRAPPER
        ceres_wrapper/complex_angle_parametrization.h 
        ceres_wrapper/ceres_manager.h )
    SET(SRCS_WRAPPER
        ceres_wrapper/complex_angle_parametrization.cpp 
        ceres_wrapper/ceres_manager.cpp)
ELSE(Ceres_FOUND)
    SET(HDRS_WRAPPER)
    SET(SRCS_WRAPPER)
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    SET(HDRS ${HDRS}
        capture_laser_2D.h
        sensor_laser_2D.h)
    SET(SRCS ${SRCS} 
        capture_laser_2D.cpp
        sensor_laser_2D.cpp)
ENDIF(laser_scan_utils_FOUND)

IF (Suitesparse_FOUND)
    ADD_SUBDIRECTORY(solver)
ENDIF(Suitesparse_FOUND)

# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS} ${SRCS_WRAPPER})

#Link the created library with ceres
IF (Ceres_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${laser_scan_utils_LIBRARY})
ENDIF (laser_scan_utils_FOUND)

#install library
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
