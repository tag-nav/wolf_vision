# NodeLinked class test
ADD_EXECUTABLE(test_node_linked test_node_linked.cpp)
TARGET_LINK_LIBRARIES(test_node_linked ${PROJECT_NAME})

# Testing Eigen Permutations
ADD_EXECUTABLE(test_permutations solver/test_permutations.cpp)
TARGET_LINK_LIBRARIES(test_permutations ${PROJECT_NAME})

IF(Suitesparse_FOUND)
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
ENDIF(Suitesparse_FOUND)

# Building and populating the wolf tree 
ADD_EXECUTABLE(test_wolf_tree test_wolf_tree.cpp)
TARGET_LINK_LIBRARIES(test_wolf_tree ${PROJECT_NAME})

# Building and populating the wolf tree from .graph file (TORO)
ADD_EXECUTABLE(test_wolf_imported_graph test_wolf_imported_graph.cpp)
TARGET_LINK_LIBRARIES(test_wolf_imported_graph ${PROJECT_NAME})

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
        ADD_EXECUTABLE(test_autodiff test_autodiff.cpp)
        TARGET_LINK_LIBRARIES(test_autodiff 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
                                
        IF(Suitesparse_FOUND)
            ADD_EXECUTABLE(test_iQR_wolf2 solver/test_iQR_wolf2.cpp)
            TARGET_LINK_LIBRARIES(test_iQR_wolf2 
                                ${pose_state_time_LIBRARIES} 
                                ${faramotics_LIBRARIES}
                                ${PROJECT_NAME})
        ENDIF(Suitesparse_FOUND)                      
    ENDIF (laser_scan_utils_FOUND)
ENDIF(faramotics_FOUND)


# Testing GPS stuff
#ADD_EXECUTABLE(test_gps_classes test_gps_classes.cpp)
#TARGET_LINK_LIBRARIES(test_gps_classes ${PROJECT_NAME})

ADD_EXECUTABLE(test_gps_wolf test_gps_wolf.cpp)
TARGET_LINK_LIBRARIES(test_gps_wolf ${PROJECT_NAME})

