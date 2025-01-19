cmake_minimum_required( VERSION 3.0 )

project( binary_data_log )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -lpthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 20 )

# Add binary_data_log_decodec.
if ( NOT TARGET lib_binary_data_log )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_binary_data_log )
endif()

# Add all 3rd libraries.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if ( NOT TARGET lib_stack_backward )
    add_subdirectory( ${SLAM_UTILITY_PATH}/3rd_libraries/backward ${PROJECT_SOURCE_DIR}/build/lib_stack_backward )
endif()

# Add visualizor.
set( SLAM_VISUALIZOR_PATH ${PROJECT_SOURCE_DIR}/../Visualizor2D/src )
if ( NOT TARGET lib_2d_visualizor )
    add_subdirectory( ${SLAM_VISUALIZOR_PATH} ${PROJECT_SOURCE_DIR}/build/lib_2d_visualizor )
endif()

# Create executable target to test binary_data_log.
add_executable( test_binary_data_log
    test/test_binary_data_log.cpp
)
target_link_libraries( test_binary_data_log
    lib_binary_data_log
    lib_2d_visualizor
    lib_stack_backward
)
