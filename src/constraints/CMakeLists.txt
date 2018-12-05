INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_CONSTRAINT ${HDRS_CONSTRAINT}
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_autodiff_distance_3D.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_autodiff_trifocal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_diff_drive.h 
  )

SET(SRCS_CONSTRAINT ${SRCS_CONSTRAINT} 
  )


# These lines always at the end
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_autodiff_apriltag.h
SET(HDRS_CONSTRAINT ${HDRS_CONSTRAINT}  PARENT_SCOPE)
SET(SRCS_CONSTRAINT ${SRCS_CONSTRAINT}   PARENT_SCOPE)

  