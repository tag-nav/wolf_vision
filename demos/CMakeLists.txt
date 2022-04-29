<<<<<<< Updated upstream

if (OpenCV_FOUND) 
  if (${OpenCV_VERSION_MAJOR} GREATER 1)
    message("-- [INFO] Found OpenCV support")
    ADD_DEFINITIONS(-DHAVE_OPENCV_H)
    SET(USE_CV true)
  else(${OpenCV_VERSION_MAJOR} GREATER 1)
    message("[WARN] OpenCV support not installed. Minimum 2.4 version required.")
    message("[WARN] Current version ${OpenCV_VERSION_MAJOR}")    
  endif(${OpenCV_VERSION_MAJOR} GREATER 1)
else(OpenCV_FOUND)
    message("[WARN] OpenCV support not installed. Minimum 2.4 version required.")
endif(OpenCV_FOUND)

ADD_EXECUTABLE(demo_processor_bundle_adjustment demo_processor_bundle_adjustment.cpp)
TARGET_LINK_LIBRARIES(demo_processor_bundle_adjustment ${PLUGIN_NAME} ${OpenCV_LIBS})
=======
ADD_EXECUTABLE(demo_visual_odometry demo_visual_odometry.cpp)
TARGET_LINK_LIBRARIES(demo_visual_odometry ${PLUGIN_NAME} ${OpenCV_LIBS})
>>>>>>> Stashed changes


