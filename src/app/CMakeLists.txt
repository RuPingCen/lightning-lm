if(BUILD_FRAMEWORK STREQUAL "DORA")
        add_executable(run_slam_online_for_dora
                run_slam_online_for_dora.cc
        )

        target_link_libraries(run_slam_online_for_dora
                ${PROJECT_NAME}.libs
                ${third_party_libs}
                $ENV{HOME}/dora/target/release/libdora_node_api_c.a
        )

        add_executable(run_loc_online_for_dora
                run_loc_online_for_dora.cc
        )

        target_link_libraries(run_loc_online_for_dora
                ${PROJECT_NAME}.libs
                ${third_party_libs}
                $ENV{HOME}/dora/target/release/libdora_node_api_c.a
        )

elseif(BUILD_FRAMEWORK STREQUAL "ROS2")

        # add_executable(run_frontend_offline
        #         run_frontend_offline.cc
        # )

        # target_link_libraries(run_frontend_offline
        #         ${PROJECT_NAME}.libs
        #         ${third_party_libs}
        # )

        # add_executable(run_loop_offline
        #         run_loop_offline.cc
        # )

        # target_link_libraries(run_loop_offline
        #         ${PROJECT_NAME}.libs
        #         ${third_party_libs}
        # )

        add_executable(run_slam_offline
                run_slam_offline.cc
        )

        target_link_libraries(run_slam_offline
                ${PROJECT_NAME}.libs
                ${third_party_libs}
        )

        add_executable(run_slam_online
                run_slam_online.cc
        )

        target_link_libraries(run_slam_online
                ${PROJECT_NAME}.libs
                ${third_party_libs}
        )

        add_executable(run_loc_offline
                run_loc_offline.cc
        )

        target_link_libraries(run_loc_offline
                ${PROJECT_NAME}.libs
                ${third_party_libs}
        )

        add_executable(run_loc_online
                run_loc_online.cc
        )

        target_link_libraries(run_loc_online
                ${PROJECT_NAME}.libs
                ${third_party_libs}
        )

        add_executable(test_ui
                test_ui.cc
        )

        target_link_libraries(test_ui
                ${PROJECT_NAME}.libs
                ${third_party_libs}
        )
        # run_slam_offline
        install(TARGETS  run_slam_offline run_slam_online run_loc_offline run_loc_online 
                DESTINATION lib/${PROJECT_NAME})
endif()