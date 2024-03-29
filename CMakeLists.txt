cmake_minimum_required( VERSION 3.10 )

project( binary_data_log )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -lpthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add binary_data_log_decodec.
if ( NOT TARGET lib_binary_data_log )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_binary_data_log )
endif()

# Add visualizor.
set( SLAM_VISUALIZOR_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D/src )
if ( NOT TARGET lib_image_visualizor )
    add_subdirectory( ${SLAM_VISUALIZOR_PATH} ${PROJECT_SOURCE_DIR}/build/lib_image_visualizor )
endif()


# Create executable target to test binary_data_log.
add_executable( test_binary_data_log
    test/test_binary_data_log.cpp
)
target_link_libraries( test_binary_data_log
    lib_binary_data_log
    lib_image_visualizor
)
