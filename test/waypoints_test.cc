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
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/compare.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/common/assertion_error.h>
#include <maliput/common/filesystem.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>

#include "assert_compare.h"

namespace maliput {
namespace {

using api::InertialPosition;
using api::LaneId;
using api::LanePosition;
using api::LaneSRange;
using api::LaneSRoute;
using api::RoadGeometry;
using maliput::api::IsEqual;
using maliput::test::AssertCompare;

GTEST_TEST(WaypointsTest, Waypoints) {
  const std::string kId = "long_start_and_end_lanes";
  const std::string kComputationPolicy = "prefer-accuracy";
  const double kScaleLength = 1.0;
  const double kLaneWidth = 6.0;
  const double kLeftShoulder = 5.0;
  const double kRightShoulder = 5.0;
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.5;
  const std::string kMultilaneYaml = [&]() {
    std::stringstream ss;
    ss << "maliput_multilane_builder:\n";
    ss << "  id: " << kId << "\n";
    ss << "  computation_policy: " << kComputationPolicy << "\n";
    ss << "  scale_length: " << kScaleLength << "\n";
    ss << "  lane_width: " << kLaneWidth << "\n";
    ss << "  left_shoulder: " << kLeftShoulder << "\n";
    ss << "  right_shoulder: " << kRightShoulder << "\n";
    ss << "  elevation_bounds: [" << kElevationBounds.min() << ", " << kElevationBounds.max() << "]\n";
    ss << "  linear_tolerance: " << kLinearTolerance << "\n";
    ss << "  angular_tolerance: " << kAngularTolerance << "\n";
    ss << R"R(
  points:
    start:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    0:
      lanes: [1, 0, 0]  # num_lanes, ref_lane, r_ref
      start: ["ref", "points.start.forward"]
      length: 5
      z_end: ["ref", [0, 0, 0]]
    1:
      lanes: [1, 0, 0]
      start: ["ref", "connections.0.end.ref.forward"]
      length: 10
      z_end: ["ref", [0, 0, 0]]
)R";

    return ss.str();
  }();

  std::unique_ptr<const RoadGeometry> rg = Load(multilane::BuilderFactory(), kMultilaneYaml);
  ASSERT_NE(rg, nullptr);
  const maliput::api::Lane* lane_one = rg->junction(0)->segment(0)->lane(0);
  const maliput::api::Lane* lane_two = rg->junction(1)->segment(0)->lane(0);

  const LaneSRange range_one{lane_one->id(), {1., 5.}};
  const LaneSRange range_two{lane_two->id(), {0., 10.}};

  const LaneSRoute route{std::vector<LaneSRange>{range_one, range_two}};
  EXPECT_EQ(route.length(), 14.0);
  const double kSampleSStep{4.0};
  std::vector<InertialPosition> waypoints = rg->SampleAheadWaypoints(route, kSampleSStep);
  std::vector<InertialPosition> expected_waypoints{InertialPosition(1.0, 0.0, 0.0), InertialPosition(5.0, 0.0, 0.0),
                                                   InertialPosition(9.0, 0.0, 0.0), InertialPosition(13.0, 0.0, 0.0),
                                                   InertialPosition(15.0, 0.0, 0.0)};

  ASSERT_EQ(waypoints.size(), 5u);
  for (unsigned int i = 0; i < waypoints.size(); ++i) {
    EXPECT_TRUE(AssertCompare(IsInertialPositionClose(waypoints[i], expected_waypoints[i], kLinearTolerance)));
  }

  const LaneSRange shorter_than_sample_step_range_one{lane_one->id(), {5., 5.}};
  const LaneSRange shorter_than_sample_step_range_two{lane_two->id(), {0., 2.}};
  std::vector<LaneSRange> shorter_ranges;
  shorter_ranges.emplace_back(shorter_than_sample_step_range_one);
  shorter_ranges.emplace_back(shorter_than_sample_step_range_two);
  const LaneSRoute shorter_route(shorter_ranges);

  EXPECT_EQ(shorter_route.length(), 2.0);
  const double kSampleSStepBiggerThanTotalRouteLength = 15.0;
  waypoints = rg->SampleAheadWaypoints(shorter_route, kSampleSStepBiggerThanTotalRouteLength);
  expected_waypoints = {
      InertialPosition(5.0, 0.0, 0.0),
      InertialPosition(7.0, 0.0, 0.0),
  };
  ASSERT_EQ(waypoints.size(), 2u);
  for (unsigned int i = 0; i < waypoints.size(); ++i) {
    EXPECT_TRUE(AssertCompare(IsInertialPositionClose(waypoints[i], expected_waypoints[i], kLinearTolerance)));
  }

  const LaneSRange non_existent_range_lane{LaneId("non-existent"), {0., 2.0}};
  const LaneSRoute non_existent_route{std::vector<LaneSRange>{non_existent_range_lane}};
  EXPECT_THROW(rg->SampleAheadWaypoints(non_existent_route, kSampleSStep), maliput::common::assertion_error);

  const double kNegativeStep = -1.0;
  EXPECT_THROW(rg->SampleAheadWaypoints(route, kNegativeStep), maliput::common::assertion_error);

  const double step_smaller_than_linear_tolerance = kLinearTolerance / 2.0;
  const LaneSRange linear_tolerance_range{lane_one->id(), {0., kLinearTolerance}};
  const LaneSRoute route_with_linear_tolerance_lane{std::vector<LaneSRange>{linear_tolerance_range}};
  expected_waypoints = {
      InertialPosition(0.0, 0.0, 0.0),
      InertialPosition(kLinearTolerance, 0.0, 0.0),
  };
  waypoints = rg->SampleAheadWaypoints(route_with_linear_tolerance_lane, step_smaller_than_linear_tolerance);
  EXPECT_EQ(waypoints.size(), 2u);
  for (unsigned int i = 0; i < waypoints.size(); ++i) {
    EXPECT_TRUE(AssertCompare(IsInertialPositionClose(waypoints[i], expected_waypoints[i], kLinearTolerance)));
  }
};

}  // namespace
}  // namespace maliput
