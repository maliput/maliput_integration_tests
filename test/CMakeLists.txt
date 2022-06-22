find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

macro(add_dependencies_to_test target)
    if (TARGET ${target})
      target_link_libraries(${target}
        maliput::api
        maliput::base
        maliput::common
        maliput::test_utilities
        maliput::routing
        maliput::utility
        maliput_object::api
        maliput_object::base
        maliput_dragway::maliput_dragway
        maliput_dragway::maliput_dragway_test_utilities
        maliput_malidrive::builder
        maliput_multilane::maliput_multilane
        maliput_multilane::test_utilities)

      # TODO(https://github.com/maliput/maliput_infrastructure/issues/222): Use ament_index for finding resources provided by the packages.
      set(MALIPUT_MULTILANE_RESOURCE_PATH "${maliput_multilane_DIR}/..")
      set(MALIPUT_MALIDRIVE_RESOURCE_PATH "${maliput_malidrive_DIR}/..")

      target_compile_definitions(${target}
        PRIVATE
          DEF_MALIDRIVE_RESOURCES="${MALIPUT_MALIDRIVE_RESOURCE_PATH}"
          DEF_MULTILANE_RESOURCES="${MALIPUT_MULTILANE_RESOURCE_PATH}"
      )
    endif()
endmacro()

set(TEST_PATH ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/MtlContent.mtl DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/ChangeOrigin.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/DefaultObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/HighlightedSegments.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/NoBranchPointsObjContent.obj DESTINATION ${TEST_PATH})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/StackedBranchPointsObjContent.obj DESTINATION ${TEST_PATH})

ament_add_gtest(derive_lane_s_routes_test derive_lane_s_routes_test.cc)
ament_add_gtest(find_lane_sequences_test find_lane_sequences_test.cc)
ament_add_gtest(generate_obj_test generate_obj_test.cc WORKING_DIRECTORY ${TEST_PATH})
ament_add_gtest(generate_string_test generate_string_test.cc)
ament_add_gtest(generate_urdf_test generate_urdf_test.cc)
ament_add_gtest(intersection_book_loader_test intersection_book_loader_test.cc)
ament_add_gmock(object_query_test object_query_test.cc)
ament_add_gtest(phase_ring_book_loader_test phase_ring_book_loader_test.cc)
ament_add_gtest(road_rulebook_loader_test road_rulebook_loader_test.cc)
ament_add_gtest(segment_analysis_test segment_analysis_test.cc)
ament_add_gtest(traffic_light_book_loader_test traffic_light_book_loader_test.cc)
ament_add_gtest(waypoints_test waypoints_test.cc)

add_dependencies_to_test(derive_lane_s_routes_test)
add_dependencies_to_test(find_lane_sequences_test)
add_dependencies_to_test(generate_obj_test)
add_dependencies_to_test(generate_string_test)
add_dependencies_to_test(generate_urdf_test)
add_dependencies_to_test(intersection_book_loader_test)
add_dependencies_to_test(object_query_test)
add_dependencies_to_test(phase_ring_book_loader_test)
add_dependencies_to_test(road_rulebook_loader_test)
add_dependencies_to_test(segment_analysis_test)
add_dependencies_to_test(traffic_light_book_loader_test)
add_dependencies_to_test(waypoints_test)
