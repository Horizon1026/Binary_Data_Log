aux_source_directory( . AUX_SRC_BINARY_DATA_LOG )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()

# Add all relative components of slam utility data type.
if( NOT TARGET lib_image )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/image ${PROJECT_SOURCE_DIR}/build/lib_image )
endif()

add_library( lib_binary_data_log ${AUX_SRC_BINARY_DATA_LOG} )
target_include_directories( lib_binary_data_log PUBLIC
    .
)
target_link_libraries( lib_binary_data_log
    lib_slam_utility_basic_type
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_operate

    lib_image
)
