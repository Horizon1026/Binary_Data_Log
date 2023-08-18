cmake_minimum_required( VERSION 3.10 )

project( hex_data_log )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -lpthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 17 )

# Add hex_data_log_decodec.
if ( NOT TARGET lib_hex_data_log_decodec )
    add_subdirectory( src ${PROJECT_SOURCE_DIR}/build/lib_hex_data_log_decodec )
endif()

# Create executable target to test hex_data_log.
add_executable( test_hex_data_log
    test/test_hex_data_log.cpp
)
target_link_libraries( test_hex_data_log
    lib_hex_data_log_decodec
)
