INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_SENSOR ${HDRS_SENSOR}
                ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_tools.h
                ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_tools.hpp
                ${CMAKE_CURRENT_SOURCE_DIR}/sensor_diff_drive.h PARENT_SCOPE)

SET(SRCS_SENSOR ${SRCS_SENSOR}
                ${CMAKE_CURRENT_SOURCE_DIR}/sensor_diff_drive.cpp PARENT_SCOPE)
