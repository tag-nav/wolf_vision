INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/capture_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/constraint_bearing.h 			
    ${CMAKE_CURRENT_SOURCE_DIR}/constraint_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/feature_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_2D.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_range_bearing.h 	
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_range_bearing.h 		
    PARENT_SCOPE
    )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hello_wolf.cpp            	
    ${CMAKE_CURRENT_SOURCE_DIR}/capture_range_bearing.cpp 	
    ${CMAKE_CURRENT_SOURCE_DIR}/feature_range_bearing.cpp 	
    ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_2D.cpp 		
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_range_bearing.cpp 	
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_range_bearing.cpp 		
    PARENT_SCOPE
    )
