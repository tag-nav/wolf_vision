# Retrieve googletest from github & compile
add_subdirectory(gtest)


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PLUGIN_NAME})

wolf_add_gtest(gtest_capture_image gtest_capture_image.cpp)
target_link_libraries(gtest_capture_image ${PLUGIN_NAME})

wolf_add_gtest(gtest_feature_point_image gtest_feature_point_image.cpp)
target_link_libraries(gtest_feature_point_image ${PLUGIN_NAME})


# wolf_add_gtest(gtest_sensor_camera gtest_sensor_camera.cpp)
# target_link_libraries(gtest_sensor_camera ${PLUGIN_NAME})

# # FactorTrifocal test
# wolf_add_gtest(gtest_factor_trifocal gtest_factor_trifocal.cpp)
# target_link_libraries(gtest_factor_trifocal ${PLUGIN_NAME})

# # FactorFeatureEpipolar test
# wolf_add_gtest(gtest_factor_epipolar gtest_factor_epipolar.cpp)
# target_link_libraries(gtest_factor_epipolar ${PLUGIN_NAME})

# # ProcessorBundleAdjustment test
# wolf_add_gtest(gtest_processor_bundle_adjustment gtest_processor_bundle_adjustment.cpp)
# target_link_libraries(gtest_processor_bundle_adjustment ${PLUGIN_NAME})

# # FactorPixelHp test
# wolf_add_gtest(gtest_factor_pixel_hp gtest_factor_pixel_hp.cpp)
# target_link_libraries(gtest_factor_pixel_hp ${PLUGIN_NAME})
