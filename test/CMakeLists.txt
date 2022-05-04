# Retrieve googletest from github & compile
add_subdirectory(gtest)

wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)

wolf_add_gtest(gtest_capture_image gtest_capture_image.cpp)

wolf_add_gtest(gtest_feature_point_image gtest_feature_point_image.cpp)

wolf_add_gtest(gtest_sensor_camera gtest_sensor_camera.cpp)

wolf_add_gtest(gtest_processor_visual_odometry gtest_processor_visual_odometry.cpp)