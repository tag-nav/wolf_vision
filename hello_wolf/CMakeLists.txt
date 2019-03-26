INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/capture_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/factor_bearing.h 			
    ${CMAKE_CURRENT_SOURCE_DIR}/factor_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/feature_range_bearing.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_2D.h 		
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_range_bearing.h 	
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_range_bearing.h 		
    )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
#    ${CMAKE_CURRENT_SOURCE_DIR}/hello_wolf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/capture_range_bearing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/feature_range_bearing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_2D.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_range_bearing.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_range_bearing.cpp
    )

ADD_EXECUTABLE(hello_wolf hello_wolf.cpp)
TARGET_LINK_LIBRARIES(hello_wolf ${PROJECT_NAME})
add_library(sensor_odom SHARED ../src/sensor/sensor_odom_2D.cpp ../src/processor/processor_odom_2D.cpp)
TARGET_LINK_LIBRARIES(sensor_odom ${PROJECT_NAME})
add_library(range_bearing SHARED sensor_range_bearing.cpp processor_range_bearing.cpp)
TARGET_LINK_LIBRARIES(range_bearing ${PROJECT_NAME})

# These lines always at the end
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}   PARENT_SCOPE    )
SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}    PARENT_SCOPE    )
