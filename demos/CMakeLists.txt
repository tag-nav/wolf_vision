ADD_EXECUTABLE(demo_visual_odometry demo_visual_odometry.cpp)
TARGET_LINK_LIBRARIES(demo_visual_odometry ${PLUGIN_NAME} ${OpenCV_LIBS})