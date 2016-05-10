# SensorFactory classes test
ADD_EXECUTABLE(test_wolf_factories test_wolf_factories.cpp)
TARGET_LINK_LIBRARIES(test_wolf_factories ${PROJECT_NAME})

# ProcessorMotion classes test
ADD_EXECUTABLE(test_motion test_motion.cpp)
TARGET_LINK_LIBRARIES(test_motion ${PROJECT_NAME})

ADD_EXECUTABLE(test_motion_2d test_motion_2d.cpp)
TARGET_LINK_LIBRARIES(test_motion_2d ${PROJECT_NAME})

# Local parametrizations classes test
ADD_EXECUTABLE(test_local_param test_local_param.cpp)
TARGET_LINK_LIBRARIES(test_local_param ${PROJECT_NAME})

# State blocks with local parametrizations test
ADD_EXECUTABLE(test_state_quaternion test_state_quaternion.cpp)
TARGET_LINK_LIBRARIES(test_state_quaternion ${PROJECT_NAME})

# NodeLinked class test
ADD_EXECUTABLE(test_node_linked test_node_linked.cpp)
TARGET_LINK_LIBRARIES(test_node_linked ${PROJECT_NAME})

# Testing virtual hierarchy -- solving diamond of death
ADD_EXECUTABLE(test_virtual_hierarchy test_virtual_hierarchy.cpp)
TARGET_LINK_LIBRARIES(test_virtual_hierarchy ${PROJECT_NAME})

# Testing Eigen Permutations
ADD_EXECUTABLE(test_permutations solver/test_permutations.cpp)
TARGET_LINK_LIBRARIES(test_permutations ${PROJECT_NAME})

# Enable Yaml config files
IF(YAMLCPP_FOUND)
    ADD_EXECUTABLE(test_yaml test_yaml.cpp)
    TARGET_LINK_LIBRARIES(test_yaml ${PROJECT_NAME})
ENDIF(YAMLCPP_FOUND)

IF(Suitesparse_FOUND)
IF (0) # These cannot compile on MacOSX -- we'll fix it some day.
    # Testing a ccolamd test
    ADD_EXECUTABLE(test_ccolamd solver/test_ccolamd.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd ${PROJECT_NAME})
    
    # Testing a blocks ccolamd test
    ADD_EXECUTABLE(test_ccolamd_blocks solver/test_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd_blocks ${PROJECT_NAME})
    
    # Testing an incremental blocks ccolamd test
    ADD_EXECUTABLE(test_incremental_ccolamd_blocks solver/test_incremental_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_incremental_ccolamd_blocks ${PROJECT_NAME})
    
    # Testing an incremental QR with block ccolamd test
    ADD_EXECUTABLE(test_iQR solver/test_iQR.cpp)
    TARGET_LINK_LIBRARIES(test_iQR ${PROJECT_NAME})
    
    # Testing QR solver test tending to wolf
    ADD_EXECUTABLE(test_iQR_wolf solver/test_iQR_wolf.cpp)
    TARGET_LINK_LIBRARIES(test_iQR_wolf ${PROJECT_NAME})
    
    # Testing SPQR simple test
    #ADD_EXECUTABLE(test_SPQR solver/test_SPQR.cpp)
    #TARGET_LINK_LIBRARIES(test_SPQR ${PROJECT_NAME})
ENDIF(0)
    
ENDIF(Suitesparse_FOUND)

# Building and populating the wolf tree 
ADD_EXECUTABLE(test_wolf_tree test_wolf_tree.cpp)
TARGET_LINK_LIBRARIES(test_wolf_tree ${PROJECT_NAME})

# Building and populating the wolf tree from .graph file (TORO)
ADD_EXECUTABLE(test_wolf_imported_graph test_wolf_imported_graph.cpp)
TARGET_LINK_LIBRARIES(test_wolf_imported_graph ${PROJECT_NAME})

# Comparing performance of wolf auto_diff and ceres auto_diff
ADD_EXECUTABLE(test_wolf_autodiffwrapper test_wolf_autodiffwrapper.cpp)
TARGET_LINK_LIBRARIES(test_wolf_autodiffwrapper ${PROJECT_NAME})

# Prunning wolf tree from .graph file (TORO)
ADD_EXECUTABLE(test_wolf_prunning test_wolf_prunning.cpp)
TARGET_LINK_LIBRARIES(test_wolf_prunning ${PROJECT_NAME})

# Comparing analytic and autodiff odometry constraints
ADD_EXECUTABLE(test_analytic_odom_constraint test_analytic_odom_constraint.cpp)
TARGET_LINK_LIBRARIES(test_analytic_odom_constraint ${PROJECT_NAME})

IF(OpenCV_FOUND)
    # Testing many things for the 3D image odometry
    ADD_EXECUTABLE(test_image test_image.cpp)
    TARGET_LINK_LIBRARIES(test_image ${PROJECT_NAME})

    # Testing opencv matching and fundamental matrix with ransac
    ADD_EXECUTABLE(test_opencv test_opencv.cpp)
    TARGET_LINK_LIBRARIES(test_opencv ${PROJECT_NAME})

    # Testing OpenCV functions for projection of points
    ADD_EXECUTABLE(test_projection_points test_projection_points.cpp)
    TARGET_LINK_LIBRARIES(test_projection_points ${PROJECT_NAME})
ENDIF(OpenCV_FOUND)

# Processor Tracker Feature test
ADD_EXECUTABLE(test_processor_tracker_feature test_processor_tracker_feature.cpp)
TARGET_LINK_LIBRARIES(test_processor_tracker_feature ${PROJECT_NAME})

# Processor Tracker Landmark test
ADD_EXECUTABLE(test_processor_tracker_landmark test_processor_tracker_landmark.cpp)
TARGET_LINK_LIBRARIES(test_processor_tracker_landmark ${PROJECT_NAME})

# IF (laser_scan_utils_FOUND)
#     ADD_EXECUTABLE(test_capture_laser_2D test_capture_laser_2D.cpp)
#     TARGET_LINK_LIBRARIES(test_capture_laser_2D ${PROJECT_NAME})
# #ENDIF (laser_scan_utils_FOUND)

IF(faramotics_FOUND)
    IF (laser_scan_utils_FOUND)
        ADD_EXECUTABLE(test_ceres_2_lasers test_ceres_2_lasers.cpp)
        TARGET_LINK_LIBRARIES(test_ceres_2_lasers 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
        ADD_EXECUTABLE(test_2_lasers_offline test_2_lasers_offline.cpp)
        TARGET_LINK_LIBRARIES(test_2_lasers_offline 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
        ADD_EXECUTABLE(test_faramotics_simulation test_faramotics_simulation.cpp)
        TARGET_LINK_LIBRARIES(test_faramotics_simulation 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
#        ADD_EXECUTABLE(test_autodiff test_autodiff.cpp)
#        TARGET_LINK_LIBRARIES(test_autodiff 
#                                ${pose_state_time_LIBRARIES} 
#                                ${faramotics_LIBRARIES}
#                                ${PROJECT_NAME})
                                
        IF(Suitesparse_FOUND)
            ADD_EXECUTABLE(test_iQR_wolf2 solver/test_iQR_wolf2.cpp)
            TARGET_LINK_LIBRARIES(test_iQR_wolf2 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
        ENDIF(Suitesparse_FOUND)                      
    ENDIF (laser_scan_utils_FOUND)
ENDIF(faramotics_FOUND)

