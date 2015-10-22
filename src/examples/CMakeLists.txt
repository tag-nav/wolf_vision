# NodeLinked class test
ADD_EXECUTABLE(test_node_linked test_node_linked.cpp)
TARGET_LINK_LIBRARIES(test_node_linked ${PROJECT_NAME})

# numeric test
ADD_EXECUTABLE(test_ceres_wrapper_numeric test_ceres_wrapper_numeric.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_numeric ${PROJECT_NAME})

# jet test
ADD_EXECUTABLE(test_ceres_wrapper_jet test_ceres_wrapper_jet.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet ${PROJECT_NAME})

# jet individual blocks test
ADD_EXECUTABLE(test_ceres_wrapper_jet_ind_block test_ceres_wrapper_jet_ind_block.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_ind_block ${PROJECT_NAME})

# jet test with 2 sensors
ADD_EXECUTABLE(test_ceres_wrapper_jet_2_sensors test_ceres_wrapper_jet_2_sensors.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_2_sensors ${PROJECT_NAME})

# jet test manager
#ADD_EXECUTABLE(test_ceres_manager test_ceres_manager.cpp)
#TARGET_LINK_LIBRARIES(test_ceres_manager ${PROJECT_NAME})

# Testing a full wolf tree avoiding template classes for NodeLinked derived classes
ADD_EXECUTABLE(test_ceres_odom_batch test_ceres_odom_batch.cpp)
TARGET_LINK_LIBRARIES(test_ceres_odom_batch ${PROJECT_NAME})

# Testing a full wolf tree avoiding template classes for NodeLinked derived classes
ADD_EXECUTABLE(test_ceres_odom_iterative test_ceres_odom_iterative.cpp)
TARGET_LINK_LIBRARIES(test_ceres_odom_iterative ${PROJECT_NAME})

# Testing Eigen Permutations
ADD_EXECUTABLE(test_permutations test_permutations.cpp)
TARGET_LINK_LIBRARIES(test_permutations ${PROJECT_NAME})


IF(Suitesparse_FOUND)
    # Testing a ccolamd test
    ADD_EXECUTABLE(test_ccolamd test_ccolamd.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd ${PROJECT_NAME})
    
    # Testing a blocks ccolamd test
    ADD_EXECUTABLE(test_ccolamd_blocks test_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd_blocks ${PROJECT_NAME})
    
    # Testing an incremental blocks ccolamd test
    ADD_EXECUTABLE(test_incremental_ccolamd_blocks test_incremental_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_incremental_ccolamd_blocks ${PROJECT_NAME})
    
    # Testing an incremental QR with block ccolamd test
    ADD_EXECUTABLE(test_iQR test_iQR.cpp)
    TARGET_LINK_LIBRARIES(test_iQR ${PROJECT_NAME})
    
    # Testing QR solver test tending to wolf
    ADD_EXECUTABLE(test_iQR_wolf test_iQR_wolf.cpp)
    TARGET_LINK_LIBRARIES(test_iQR_wolf ${PROJECT_NAME})
    
    # Testing SPQR simple test
    #ADD_EXECUTABLE(test_SPQR test_SPQR.cpp)
    #TARGET_LINK_LIBRARIES(test_SPQR ${PROJECT_NAME})
ENDIF(Suitesparse_FOUND)

# Building and populating the wolf tree 
# ADD_EXECUTABLE(test_wolf_tree test_wolf_tree.cpp)
# TARGET_LINK_LIBRARIES(test_wolf_tree ${PROJECT_NAME})

# test ceres manager, wolf manager and wolf tree
#ADD_EXECUTABLE(test_ceres_manager_tree test_ceres_manager_tree.cpp)
#TARGET_LINK_LIBRARIES(test_ceres_manager_tree ${PROJECT_NAME})

# test ceres covariance
#ADD_EXECUTABLE(test_ceres_covariance test_ceres_covariance.cpp)
#TARGET_LINK_LIBRARIES(test_ceres_covariance ${PROJECT_NAME})

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
                                
        IF(Suitesparse_FOUND)
            ADD_EXECUTABLE(test_iQR_wolf2 test_iQR_wolf2.cpp)
            TARGET_LINK_LIBRARIES(test_iQR_wolf2 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
        ENDIF(Suitesparse_FOUND)                      
    ENDIF (laser_scan_utils_FOUND)
ENDIF(faramotics_FOUND)

# Testing a eigen quaternion
ADD_EXECUTABLE(test_state_quaternion test_state_quaternion.cpp)
TARGET_LINK_LIBRARIES(test_state_quaternion ${PROJECT_NAME})

