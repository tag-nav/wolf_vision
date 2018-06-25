INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_CAPTURE ${HDRS_CAPTURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_velocity.h
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_wheel_joint_position.h 
  )

SET(SRCS_CAPTURE ${SRCS_CAPTURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_velocity.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_wheel_joint_position.cpp 
  )






# These lines always at the end
SET(HDRS_CAPTURE ${HDRS_CAPTURE}  PARENT_SCOPE)
SET(SRCS_CAPTURE ${SRCS_CAPTURE}  PARENT_SCOPE)
