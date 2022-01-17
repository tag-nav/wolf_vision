# Retrieve googletest from github & compile
add_subdirectory(gtest)


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

############# USE THIS TEST AS AN EXAMPLE #################
#                                                         #
# Create a specific test executable for gtest_example     #
wolf_add_gtest(gtest_example gtest_example.cpp)           #
target_link_libraries(gtest_example ${PLUGIN_NAME})      #
#                                                         #
###########################################################

# Pinhole test
wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PLUGIN_NAME})

wolf_add_gtest(gtest_sensor_camera gtest_sensor_camera.cpp)
target_link_libraries(gtest_sensor_camera ${PLUGIN_NAME})

# FactorTrifocal test
wolf_add_gtest(gtest_factor_trifocal gtest_factor_trifocal.cpp)
target_link_libraries(gtest_factor_trifocal ${PLUGIN_NAME})

# FactorFeatureEpipolar test
wolf_add_gtest(gtest_factor_epipolar gtest_factor_epipolar.cpp)
target_link_libraries(gtest_factor_epipolar ${PLUGIN_NAME})

# ProcessorFeatureTrifocal test
wolf_add_gtest(gtest_processor_tracker_feature_trifocal gtest_processor_tracker_feature_trifocal.cpp)
target_link_libraries(gtest_processor_tracker_feature_trifocal ${PLUGIN_NAME})

# ProcessorBundleAdjustment test
wolf_add_gtest(gtest_processor_bundle_adjustment gtest_processor_bundle_adjustment.cpp)
target_link_libraries(gtest_processor_bundle_adjustment ${PLUGIN_NAME})

# FactorPixelHp test
wolf_add_gtest(gtest_factor_pixel_hp gtest_factor_pixel_hp.cpp)
target_link_libraries(gtest_factor_pixel_hp ${PLUGIN_NAME})
