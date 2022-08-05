// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/common/filesystem.h>
#include <maliput/math/bounding_box.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>
#include <maliput_object/api/object_query.h>
#include <maliput_object/base/simple_object_query.h>
#include <maliput_object/test_utilities/mock.h>

namespace maliput {
namespace {

#define EXPECT_FOUND_IN(item, expected_items)                                                       \
  do {                                                                                              \
    EXPECT_NE(expected_items.end(), std::find(expected_items.begin(), expected_items.end(), item)); \
  } while (0)

// Verifies if `lane_s_route_a` and `lane_s_route_b` are equal based on a `tolerance`.
// TODO(https://github.com/ToyotaResearchInstitute/maliput/issues/463): Use "maliput/test_utilities/rules_compare.h"
// instead when zone(ranges) comparison includes tolerance.
inline ::testing::AssertionResult IsEqual(const maliput::api::LaneSRoute& lane_s_route_a,
                                          const maliput::api::LaneSRoute& lane_s_route_b, double tolerance) {
  for (const auto& lane_s_range_a : lane_s_route_a.ranges()) {
    auto lane_s_range_b = std::find_if(lane_s_route_b.ranges().begin(), lane_s_route_b.ranges().end(),
                                       [&lane_s_range_a](const maliput::api::LaneSRange& lane_s_range_b) {
                                         return lane_s_range_b.lane_id() == lane_s_range_a.lane_id();
                                       });
    if (lane_s_range_b == lane_s_route_b.ranges().end()) {
      return ::testing::AssertionFailure() << "LaneSRoute are different: " << lane_s_range_a.lane_id().string()
                                           << " lane id doesn't belong to the other lane_s_route";
    }
    if (std::abs(lane_s_range_a.s_range().s0() - lane_s_range_b->s_range().s0()) >= tolerance) {
      return ::testing::AssertionFailure()
             << "Ranges(s0) are different for lane id: " << lane_s_range_a.lane_id().string()
             << ": s0: " << lane_s_range_a.s_range().s0() << " vs " << lane_s_range_b->s_range().s0();
    }
    if (std::abs(lane_s_range_a.s_range().s1() - lane_s_range_b->s_range().s1()) >= tolerance) {
      return ::testing::AssertionFailure()
             << "Ranges(s1) are different for lane id: " << lane_s_range_a.lane_id().string()
             << ": s1: " << lane_s_range_a.s_range().s1() << " vs " << lane_s_range_b->s_range().s1();
    }
  }
  return ::testing::AssertionSuccess();
}

using api::RoadNetwork;

static constexpr char kMalidriveResourcesPath[] = DEF_MALIDRIVE_RESOURCES;

struct SimpleObjectQueryResults {
  maliput::math::Vector3 box_position{};
  maliput::math::Vector3 box_size{};
  maliput::math::RollPitchYaw box_orientation{};
  std::vector<maliput::api::LaneId> expected_intersected_lane_ids;
  std::vector<maliput::api::LaneId> expected_disjointed_lane_ids;
};

std::vector<SimpleObjectQueryResults> InstantiateSimpleObjectQueryParameters() {
  return {
      // At the middle of the lane 0_0_1.
      {{25., 1.75, 0.},
       {0.5, 0.5, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_1"}},
       {maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"}, maliput::api::LaneId{"1_0_-1"},
        maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"}, maliput::api::LaneId{"4_0_1"},
        maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"},
        maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}}},
      // At the start of the segment 0_0.
      {{0., 0., 0.},
       {5., 5., 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"0_0_1"}},
       {maliput::api::LaneId{"1_0_1"}, maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"2_0_1"},
        maliput::api::LaneId{"2_0_-1"}, maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"},
        maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"},
        maliput::api::LaneId{"9_0_-1"}}},
      // At the middle of the T intersection avoiding 7_0_-1 and 9_0_-1
      {{50., 0, 0.},
       {0.5, 0.5, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"},
        maliput::api::LaneId{"8_0_-1"}},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"},
        maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"9_0_-1"}}},
      // At the middle of the T intersection intersecting all the lanes in the intersection.
      {{50., 0, 0.},
       {3., 3., 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"},
        maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"}}},
      // At the middle of the T intersection. Extruded in y direction.
      {{50., 0, 0.},
       {3, 10., 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"}, maliput::api::LaneId{"4_0_1"},
        maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"},
        maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}}},
      // At the middle of the T intersection. Extruded in x direction.
      {{50., 0, 0.},
       {10, 3, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"},
        maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"},
        maliput::api::LaneId{"9_0_-1"}},
       {maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"}}},
      // At the middle of the T intersection. Extruded in both x and y direction. All lanes are intersected.
      {{50., 0, 0.},
       {10, 10, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"},
        maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"},
        maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}},
       {}},
  };
}

class SimpleObjectQueryFindLanesTest : public ::testing::TestWithParam<SimpleObjectQueryResults> {
 public:
  SimpleObjectQueryFindLanesTest()
      : filepath_(std::string(kMalidriveResourcesPath) + "/resources/odr/TShapeRoad.xodr") {}

  void SetUp() override {
    road_network_ = malidrive::builder::RoadNetworkBuilder(configuration_)();
    dut_ = std::make_unique<maliput::object::SimpleObjectQuery>(road_network_.get(), &object_book);
  }

  const SimpleObjectQueryResults results_ = GetParam();
  const maliput::object::api::Object<maliput::math::Vector3>::Id kId{"kId"};
  const double kTolerance{1e-6};
  const std::string filepath_;
  const std::map<std::string, std::string> configuration_{{malidrive::builder::params::kOpendriveFile, filepath_}};
  const maliput::object::test_utilities::MockObjectBook<maliput::math::Vector3> object_book;
  std::unique_ptr<const RoadNetwork> road_network_;
  std::unique_ptr<maliput::object::api::ObjectQuery> dut_;
};

TEST_P(SimpleObjectQueryFindLanesTest, FindOverlappingLanes) {
  const object::api::Object<maliput::math::Vector3> object{
      kId,
      {},
      std::make_unique<math::BoundingBox>(results_.box_position, results_.box_size, results_.box_orientation,
                                            kTolerance)};
  const auto intersected_lanes =
      dut_->FindOverlappingLanesIn(&object, maliput::math::OverlappingType::kIntersected);
  EXPECT_EQ(results_.expected_intersected_lane_ids.size(), intersected_lanes.size());
  for (const auto lane : intersected_lanes) {
    EXPECT_FOUND_IN(lane->id(), results_.expected_intersected_lane_ids);
  }

  const auto disjointed_lanes =
      dut_->FindOverlappingLanesIn(&object, maliput::math::OverlappingType::kDisjointed);
  EXPECT_EQ(results_.expected_disjointed_lane_ids.size(), disjointed_lanes.size());
  for (const auto lane : disjointed_lanes) {
    EXPECT_FOUND_IN(lane->id(), results_.expected_disjointed_lane_ids);
  }

  // TODO(#https://github.com/ToyotaResearchInstitute/maliput_object/issues/22): Adds test for
  // OverlappingType::kContained once that case is implemented.
  EXPECT_THROW(dut_->FindOverlappingLanesIn(&object, maliput::math::OverlappingType::kContained),
               maliput::common::assertion_error);
}

INSTANTIATE_TEST_CASE_P(SimpleObjectQueryGroupTest, SimpleObjectQueryFindLanesTest,
                        ::testing::ValuesIn(InstantiateSimpleObjectQueryParameters()));

struct SimpleObjectQueryRouteResults {
  struct BoxConfig {
    maliput::math::Vector3 position{};
    maliput::math::Vector3 size{};
    maliput::math::RollPitchYaw orientation{};
  };
  BoxConfig origin{};
  BoxConfig target{};
  std::optional<maliput::api::LaneSRoute> expected_lane_s_route;
};

std::vector<SimpleObjectQueryRouteResults> InstantiateSimpleObjectQueryRouteParameters() {
  return {
      {{{2., -2., 0.}, {1., 1., 1.}, {0., 0., 0.}},
       {{48., -28., 0.}, {1., 1., 1.}, {0., 0., 0.}},
       {
           std::make_optional<maliput::api::LaneSRoute>({
               maliput::api::LaneSRange{maliput::api::LaneId{"0_0_-1"},
                                        maliput::api::SRange{1.999999999999, 45.99999999999}},
               maliput::api::LaneSRange{maliput::api::LaneId{"9_0_-1"}, maliput::api::SRange{0., 3.5635908347154772}},
               maliput::api::LaneSRange{maliput::api::LaneId{"2_0_1"},
                                        maliput::api::SRange{45.999999999999, 21.999999999999}},
           }),
       }}};
}

class SimpleObjectQueryRouteTest : public ::testing::TestWithParam<SimpleObjectQueryRouteResults> {
 public:
  SimpleObjectQueryRouteTest() : filepath_(std::string(kMalidriveResourcesPath) + "/resources/odr/TShapeRoad.xodr") {}

  void SetUp() override {
    road_network_ = malidrive::builder::RoadNetworkBuilder(configuration_)();
    dut_ = std::make_unique<maliput::object::SimpleObjectQuery>(road_network_.get(), &object_book);
  }

  const SimpleObjectQueryRouteResults results_ = GetParam();
  const maliput::object::api::Object<maliput::math::Vector3>::Id kId{"kId"};
  const double kTolerance{1e-6};
  const std::string filepath_;
  const std::map<std::string, std::string> configuration_{{malidrive::builder::params::kOpendriveFile, filepath_}};
  const maliput::object::test_utilities::MockObjectBook<maliput::math::Vector3> object_book;
  std::unique_ptr<const RoadNetwork> road_network_;
  std::unique_ptr<maliput::object::api::ObjectQuery> dut_;
};

TEST_P(SimpleObjectQueryRouteTest, Route) {
  const object::api::Object<maliput::math::Vector3> origin_obj{
      kId,
      {},
      std::make_unique<math::BoundingBox>(results_.origin.position, results_.origin.size, results_.origin.orientation,
                                            kTolerance)};
  const object::api::Object<maliput::math::Vector3> target_obj{
      kId,
      {},
      std::make_unique<math::BoundingBox>(results_.target.position, results_.target.size, results_.target.orientation,
                                            kTolerance)};

  const auto lane_s_route = dut_->Route(&origin_obj, &target_obj);
  ASSERT_TRUE(lane_s_route.has_value());
  EXPECT_TRUE(IsEqual(lane_s_route.value(), results_.expected_lane_s_route.value(), kTolerance));
}

INSTANTIATE_TEST_CASE_P(SimpleObjectQueryGroupTest, SimpleObjectQueryRouteTest,
                        ::testing::ValuesIn(InstantiateSimpleObjectQueryRouteParameters()));

}  // namespace
}  // namespace maliput
