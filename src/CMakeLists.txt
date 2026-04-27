if(BUILD_FRAMEWORK STREQUAL "DORA")

        add_library(${PROJECT_NAME}.libs SHARED
                common/nav_state.cc
                common/constant.h
                common/options.cc  #for  ros 
                common/params.cc

                io/yaml_io.cc
                io/file_io.cc

                core/lio/eskf.cc
                core/lio/laser_mapping.cc          #for  ros 
              #  core/lio/pointcloud_preprocess.cc  #for  ros 
                core/loop_closing/loop_closing.cc  

                core/g2p5/g2p5_map.cc  #for  ros 
                core/g2p5/g2p5.cc
                core/g2p5/g2p5_subgrid.cc

                core/maps/tiled_map.cc
                core/maps/tiled_map_chunk.cc

                core/localization/localization_result.cc #for  ros 
                core/localization/lidar_loc/lidar_loc.cc #for  ros 
                core/localization/lidar_loc/pclomp/ndt_omp.cpp
                core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

                core/localization/pose_graph/pgo.cc
                core/localization/pose_graph/pgo_impl.cc
                core/localization/pose_graph/pose_extrapolator.cc
                core/localization/localization.cpp

                utils/timer.cc
                utils/pointcloud_utils.cc

                ui/pangolin_window.cc
                ui/pangolin_window_impl.cc
                ui/ui_car.cc
                ui/ui_cloud.cc
                ui/ui_trajectory.cc

                core/system/slam.cc
                core/system/loc_system.cc

               # wrapper/bag_io.cc   #for  ros 
        )
        find_package(PCL REQUIRED)
        find_package(Eigen3 QUIET)
        #find_package(PkgConfig)
        #pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

        if(NOT EIGEN3_FOUND)
                # Fallback to cmake_modules
                find_package(cmake_modules REQUIRED)
                find_package(Eigen REQUIRED)
                set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
                set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
                # Possibly map additional variables to the EIGEN3_ prefix.
        else()
                set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
        endif()
 
        include_directories(
                ${PCL_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIRS}
                $ENV{HOME}/dora/apis/c/node #dora的头文件路径 node_api.h
                $ENV{HOME}/dora/apis/c/operator
                # $ENV{HOME}/dora_project/dora/examples/c++-ros2-dataflow/build/ #C++ros的头文件路径
                )
         
        include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
        add_subdirectory(core/miao)

        target_link_libraries(${PROJECT_NAME}.libs
                ${third_party_libs}
                ${PCL_LIBRARIES}
                miao.core
                miao.utils
               #livox_interfaces2__rosidl_typesupport_cpp
                #${PROJECT_NAME}__rosidl_typesupport_cpp
        )
 

elseif(BUILD_FRAMEWORK STREQUAL "ROS2")

        add_library(${PROJECT_NAME}.libs SHARED
                common/nav_state.cc
                common/constant.h
                common/options.cc  #for  ros 
                common/params.cc

                io/yaml_io.cc
                io/file_io.cc

                core/lio/eskf.cc
                core/lio/laser_mapping.cc          #for  ros 
                core/lio/pointcloud_preprocess.cc  #for  ros 
                core/loop_closing/loop_closing.cc  

                core/g2p5/g2p5_map.cc  #for  ros 
                core/g2p5/g2p5.cc
                core/g2p5/g2p5_subgrid.cc

                core/maps/tiled_map.cc
                core/maps/tiled_map_chunk.cc

                core/localization/localization_result.cc #for  ros 
                core/localization/lidar_loc/lidar_loc.cc #for  ros 
                core/localization/lidar_loc/pclomp/ndt_omp.cpp
                core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

                core/localization/pose_graph/pgo.cc
                core/localization/pose_graph/pgo_impl.cc
                core/localization/pose_graph/pose_extrapolator.cc
                core/localization/localization.cpp

                utils/timer.cc
                utils/pointcloud_utils.cc

                ui/pangolin_window.cc
                ui/pangolin_window_impl.cc
                ui/ui_car.cc
                ui/ui_cloud.cc
                ui/ui_trajectory.cc

                core/system/slam.cc
                core/system/loc_system.cc

                wrapper/bag_io.cc   #for  ros 
        )

        ament_target_dependencies(
                ${PROJECT_NAME}.libs
                "rclcpp"
                "std_msgs"
                "geometry_msgs"
                "sensor_msgs"
                "nav_msgs"
                "std_srvs"
                "message_filters"
                "pcl_conversions"
                "tf2_ros"
                "tf2"
        )

        add_dependencies(${PROJECT_NAME}.libs
                livox_interfaces2__rosidl_typesupport_cpp
                ${PROJECT_NAME}__rosidl_typesupport_cpp
        )

        include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
        add_subdirectory(core/miao)

        target_link_libraries(${PROJECT_NAME}.libs
                ${third_party_libs}
                ${PCL_LIBRARIES}
                miao.core
                miao.utils
                livox_interfaces2__rosidl_typesupport_cpp
                ${PROJECT_NAME}__rosidl_typesupport_cpp
        )

        install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils
                DESTINATION lib)
endif()

add_subdirectory(app)
