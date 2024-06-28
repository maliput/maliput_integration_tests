// BSD 3-Clause License
//
// Copyright (c) 2024, Woven by Toyota. All rights reserved.
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
#include "maliput/base/distance_router.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/compare.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/common/filesystem.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/routing/phase.h>
#include <maliput/routing/route.h>
#include <maliput/routing/router.h>
#include <maliput/routing/routing_constraints.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_malidrive/loader/loader.h>

#include "assert_compare.h"

namespace maliput {
namespace test {
namespace {

using maliput::api::IsEqual;
using maliput::api::IsLanePositionClose;

static constexpr char kMalidriveResourcesPath[] = DEF_MALIDRIVE_RESOURCES;

void CheckRoutingPhase(const routing::Phase& phase, int index, double tolerance,
                       const std::vector<api::RoadPosition>& start_pos, const std::vector<api::RoadPosition>& end_pos,
                       const std::vector<api::LaneSRange>& lane_s_ranges) {
  ASSERT_EQ(index, phase.index()) << "phase " << index << " with indices are not equal.";
  ASSERT_EQ(tolerance, phase.lane_s_range_tolerance()) << "phase " << index << " with tolerances are not equal.";
  ASSERT_EQ(start_pos.size(), phase.start_positions().size())
      << "phase " << index << " with start position sizes are not equal.";
  for (size_t i = 0; i < start_pos.size(); ++i) {
    ASSERT_EQ(start_pos[i].lane, phase.start_positions()[i].lane)
        << "phase " << index << " with index " << i << " with mismatched RoadPositions.";
    ASSERT_TRUE(AssertCompare(IsLanePositionClose(start_pos[i].pos, phase.start_positions()[i].pos, tolerance)))
        << "phase " << index << " with index " << i << " with mismatched RoadPositions.";
  }
  ASSERT_EQ(end_pos.size(), phase.end_positions().size())
      << "phase " << index << " with end position sizes are not equal.";
  for (size_t i = 0; i < start_pos.size(); ++i) {
    ASSERT_EQ(end_pos[i].lane, phase.end_positions()[i].lane)
        << "phase " << index << " with index " << i << " with mismatched RoadPositions.";
    ASSERT_TRUE(AssertCompare(IsLanePositionClose(end_pos[i].pos, phase.end_positions()[i].pos, tolerance)))
        << "phase " << index << " with index " << i << " with mismatched RoadPositions.";
  }
  ASSERT_EQ(lane_s_ranges.size(), phase.lane_s_ranges().size())
      << "phase " << index << " with LaneSRanges sizes are not equal.";
  for (size_t i = 0; i < lane_s_ranges.size(); ++i) {
    ASSERT_EQ(lane_s_ranges[i].lane_id().string(), phase.lane_s_ranges()[i].lane_id().string())
        << "phase " << index << " with index " << i << " with mismatched LaneIds.";
    ASSERT_NEAR(lane_s_ranges[i].s_range().s0(), phase.lane_s_ranges()[i].s_range().s0(), tolerance)
        << "phase " << index << " with index " << i << " with mismatched SRange::s0.";
    ASSERT_NEAR(lane_s_ranges[i].s_range().s1(), phase.lane_s_ranges()[i].s_range().s1(), tolerance)
        << "phase " << index << " with index " << i << " with mismatched SRange::s1.";
  }
}

class TShapeRoadRoutingTest : public ::testing::Test {
 public:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{1.0};
  //@}
  static constexpr routing::RoutingConstraints kDefaultRoutingConstraints{};
  static constexpr routing::RoutingConstraints kSmallPhaseCostConstraint{
      true /* allow_lane_switch */, std::optional<double>{1.} /* max_phase_cost */, std::nullopt /* max_route_cost */
  };
  static constexpr routing::RoutingConstraints kSmallRouteCostConstraint{
      true /* allow_lane_switch */, std::nullopt /* max_phase_cost */, std::optional<double>{1.} /* max_route_cost */
  };
  const std::string kTShapeRoadFilePath{std::string(kMalidriveResourcesPath) +
                                        std::string("/resources/odr/TShapeRoad.xodr")};
  const api::LaneId kLaneId_0_0_1{"0_0_1"};
  const api::LaneId kLaneId_0_0_m1{"0_0_-1"};
  const api::LaneId kLaneId_1_0_1{"1_0_1"};
  const api::LaneId kLaneId_1_0_m1{"1_0_-1"};
  const api::LaneId kLaneId_4_0_1{"4_0_1"};
  const api::LaneId kLaneId_5_0_m1{"5_0_-1"};

  void SetUp() override {
    std::map<std::string, std::string> road_network_configuration;
    road_network_configuration.emplace(malidrive::builder::params::kRoadGeometryId, "malidrive_rg");
    road_network_configuration.emplace(malidrive::builder::params::kOpendriveFile, kTShapeRoadFilePath);
    road_network_configuration.emplace(malidrive::builder::params::kLinearTolerance, std::to_string(kLinearTolerance));
    road_network_configuration.emplace(malidrive::builder::params::kAngularTolerance,
                                       std::to_string(kAngularTolerance));
    road_network_configuration.emplace(malidrive::builder::params::kScaleLength, std::to_string(kScaleLength));
    road_network_configuration.emplace(malidrive::builder::params::kInertialToBackendFrameTranslation, "{0., 0., 0.}");
    road_network_ = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);

    dut_ = std::make_unique<maliput::DistanceRouter>(*road_network_, kLinearTolerance);

    lane_0_0_1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_0_0_1);
    lane_0_0_m1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_0_0_m1);
    lane_1_0_1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_1_0_1);
    lane_1_0_m1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_1_0_m1);
    lane_4_0_1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_4_0_1);
    lane_5_0_m1_ = road_network_->road_geometry()->ById().GetLane(kLaneId_5_0_m1);

    ASSERT_NE(lane_0_0_1_, nullptr);
    ASSERT_NE(lane_0_0_m1_, nullptr);
    ASSERT_NE(lane_1_0_1_, nullptr);
    ASSERT_NE(lane_1_0_m1_, nullptr);
    ASSERT_NE(lane_4_0_1_, nullptr);
    ASSERT_NE(lane_5_0_m1_, nullptr);
  }

  std::unique_ptr<api::RoadNetwork> road_network_{};
  std::unique_ptr<routing::Router> dut_{};
  const api::Lane* lane_0_0_1_{};
  const api::Lane* lane_0_0_m1_{};
  const api::Lane* lane_1_0_1_{};
  const api::Lane* lane_1_0_m1_{};
  const api::Lane* lane_4_0_1_{};
  const api::Lane* lane_5_0_m1_{};
};

// Defines the test cases for the TShapeRoad where the start and end positions are on the very same Lane.
class RoutingInTheSameLaneTest : public TShapeRoadRoutingTest {
 public:
  void SetUp() override {
    TShapeRoadRoutingTest::SetUp();
    start_ = api::RoadPosition(lane_0_0_1_, api::LanePosition(1., 0., 0.));
    end_ = api::RoadPosition(lane_0_0_1_, api::LanePosition(10., 0., 0.));
  }

  api::RoadPosition start_;
  api::RoadPosition end_;
};

// No constraints are provided and the only possible route is returned.
TEST_F(RoutingInTheSameLaneTest, WithDefaultConstraintsReturnsTheLane) {
  const std::vector<api::LaneSRange> kPhaseLaneSRanges{api::LaneSRange(kLaneId_0_0_m1, api::SRange(1., 10.)),
                                                       api::LaneSRange(kLaneId_0_0_1, api::SRange(1., 10.))};

  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kDefaultRoutingConstraints);

  ASSERT_EQ(1u, routes.size());
  const routing::Route& route = routes[0];
  ASSERT_EQ(1, route.size());
  const routing::Phase& phase = route.Get(0);
  CheckRoutingPhase(phase, 0, kLinearTolerance, std::vector<api::RoadPosition>{start_},
                    std::vector<api::RoadPosition>{end_}, kPhaseLaneSRanges);
}

// The maximum cost of the phase is smaller than the solution's phase cost, so no routes can be found.
TEST_F(RoutingInTheSameLaneTest, WithConstrainedPhaseCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallPhaseCostConstraint);

  ASSERT_TRUE(routes.empty());
}

// The maximum cost of the route is smaller than the solution's phase cost, so no routes can be found.
TEST_F(RoutingInTheSameLaneTest, WithConstrainedRouteCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallRouteCostConstraint);

  ASSERT_TRUE(routes.empty());
}

// Defines the test cases for the TShapeRoad where the start and end positions are on the extremes of the
// three roads aligned in a straight line.
class DriveBackwardStraightOverMultipleLanesTest : public TShapeRoadRoutingTest {
 public:
  const std::vector<api::LaneSRange> kRoute0Phase0LaneSRanges{api::LaneSRange{kLaneId_1_0_m1, api::SRange{1., 0.}},
                                                              api::LaneSRange{kLaneId_1_0_1, api::SRange{1., 0.}}};
  const std::vector<api::LaneSRange> kRoute0Phase1LaneSRanges{api::LaneSRange{kLaneId_4_0_1, api::SRange{8., 0.}}};
  const std::vector<api::LaneSRange> kRoute0Phase2LaneSRanges{api::LaneSRange{kLaneId_0_0_m1, api::SRange{46., 10.}},
                                                              api::LaneSRange{kLaneId_0_0_1, api::SRange{46., 10.}}};
  const std::vector<api::LaneSRange> kRoute1Phase0LaneSRanges{api::LaneSRange{kLaneId_1_0_m1, api::SRange{1., 0.}},
                                                              api::LaneSRange{kLaneId_1_0_1, api::SRange{1., 0.}}};
  const std::vector<api::LaneSRange> kRoute1Phase1LaneSRanges{api::LaneSRange{kLaneId_5_0_m1, api::SRange{8., 0.}}};
  const std::vector<api::LaneSRange> kRoute1Phase2LaneSRanges{api::LaneSRange{kLaneId_0_0_m1, api::SRange{46., 10.}},
                                                              api::LaneSRange{kLaneId_0_0_1, api::SRange{46., 10.}}};

  void SetUp() override {
    TShapeRoadRoutingTest::SetUp();
    start_ = api::RoadPosition(lane_1_0_1_, api::LanePosition(1., 0., 0.));
    end_ = api::RoadPosition(lane_0_0_1_, api::LanePosition(10., 0., 0.));
    // Route 0
    route_0_phase_0_start_ = start_;
    route_0_phase_0_end_ = api::RoadPosition(lane_1_0_1_, api::LanePosition(0., 0., 0.));
    route_0_phase_1_start_ = api::RoadPosition(lane_4_0_1_, api::LanePosition(8., 0., 0.));
    route_0_phase_1_end_ = api::RoadPosition(lane_4_0_1_, api::LanePosition(0., 0., 0.));
    route_0_phase_2_start_ = api::RoadPosition(lane_0_0_1_, api::LanePosition(46., 0., 0.));
    route_0_phase_2_end_ = end_;
    // Route 1
    route_1_phase_0_start_ = start_;
    route_1_phase_0_end_ = api::RoadPosition(lane_1_0_m1_, api::LanePosition(0., 0., 0.));
    route_1_phase_1_start_ = api::RoadPosition(lane_5_0_m1_, api::LanePosition(8., 0., 0.));
    route_1_phase_1_end_ = api::RoadPosition(lane_5_0_m1_, api::LanePosition(0., 0., 0.));
    route_1_phase_2_start_ = api::RoadPosition(lane_0_0_m1_, api::LanePosition(46., 0., 0.));
    route_1_phase_2_end_ = end_;
  }

  api::RoadPosition start_;
  api::RoadPosition end_;
  api::RoadPosition route_0_phase_0_start_;
  api::RoadPosition route_0_phase_0_end_;
  api::RoadPosition route_0_phase_1_start_;
  api::RoadPosition route_0_phase_1_end_;
  api::RoadPosition route_0_phase_2_start_;
  api::RoadPosition route_0_phase_2_end_;
  api::RoadPosition route_1_phase_0_start_;
  api::RoadPosition route_1_phase_0_end_;
  api::RoadPosition route_1_phase_1_start_;
  api::RoadPosition route_1_phase_1_end_;
  api::RoadPosition route_1_phase_2_start_;
  api::RoadPosition route_1_phase_2_end_;
};

// No constraints are provided and the only possible route is returned.
TEST_F(DriveBackwardStraightOverMultipleLanesTest, WithDefaultConstraintsReturnsRouteWithThreePhases) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kDefaultRoutingConstraints);

  ASSERT_EQ(2u, routes.size());
  {
    const routing::Route& route = routes[0];
    ASSERT_EQ(3, route.size());
    const routing::Phase& phase_0 = route.Get(0);
    const routing::Phase& phase_1 = route.Get(1);
    const routing::Phase& phase_2 = route.Get(2);
    CheckRoutingPhase(phase_0, 0, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_0_start_},
                      std::vector<api::RoadPosition>{route_0_phase_0_end_}, kRoute0Phase0LaneSRanges);
    CheckRoutingPhase(phase_1, 1, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_1_start_},
                      std::vector<api::RoadPosition>{route_0_phase_1_end_}, kRoute0Phase1LaneSRanges);
    CheckRoutingPhase(phase_2, 2, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_2_start_},
                      std::vector<api::RoadPosition>{route_0_phase_2_end_}, kRoute0Phase2LaneSRanges);
  }
  {
    const routing::Route& route = routes[1];
    ASSERT_EQ(3, route.size());
    const routing::Phase& phase_0 = route.Get(0);
    const routing::Phase& phase_1 = route.Get(1);
    const routing::Phase& phase_2 = route.Get(2);
    CheckRoutingPhase(phase_0, 0, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_0_start_},
                      std::vector<api::RoadPosition>{route_1_phase_0_end_}, kRoute1Phase0LaneSRanges);
    CheckRoutingPhase(phase_1, 1, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_1_start_},
                      std::vector<api::RoadPosition>{route_1_phase_1_end_}, kRoute1Phase1LaneSRanges);
    CheckRoutingPhase(phase_2, 2, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_2_start_},
                      std::vector<api::RoadPosition>{route_1_phase_2_end_}, kRoute1Phase2LaneSRanges);
  }
}

// The maximum cost of the phase is smaller than the solution's phase cost, so no routes can be found.
TEST_F(DriveBackwardStraightOverMultipleLanesTest, WithConstrainedPhaseCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallPhaseCostConstraint);

  ASSERT_TRUE(routes.empty());
}

// The maximum cost of the route is smaller than the solution's phase cost, so no routes can be found.
TEST_F(DriveBackwardStraightOverMultipleLanesTest, WithConstrainedRouteCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallRouteCostConstraint);

  ASSERT_TRUE(routes.empty());
}

// Defines the test cases for the TShapeRoad where the start and end positions are on the extremes of the
// three roads aligned in a straight line.
class DriveForwardStraightOverMultipleLanesTest : public TShapeRoadRoutingTest {
 public:
  const std::vector<api::LaneSRange> kRoute0Phase0LaneSRanges{api::LaneSRange{kLaneId_0_0_m1, api::SRange{1., 46.}},
                                                              api::LaneSRange{kLaneId_0_0_1, api::SRange{1., 46.}}};
  const std::vector<api::LaneSRange> kRoute0Phase1LaneSRanges{api::LaneSRange{kLaneId_4_0_1, api::SRange{0., 8.}}};
  const std::vector<api::LaneSRange> kRoute0Phase2LaneSRanges{api::LaneSRange{kLaneId_1_0_m1, api::SRange{0., 10.}},
                                                              api::LaneSRange{kLaneId_1_0_1, api::SRange{0., 10.}}};
  const std::vector<api::LaneSRange> kRoute1Phase0LaneSRanges{api::LaneSRange{kLaneId_0_0_m1, api::SRange{1., 46.}},
                                                              api::LaneSRange{kLaneId_0_0_1, api::SRange{1., 46.}}};
  const std::vector<api::LaneSRange> kRoute1Phase1LaneSRanges{api::LaneSRange{kLaneId_5_0_m1, api::SRange{0., 8.}}};
  const std::vector<api::LaneSRange> kRoute1Phase2LaneSRanges{api::LaneSRange{kLaneId_1_0_m1, api::SRange{0., 10.}},
                                                              api::LaneSRange{kLaneId_1_0_1, api::SRange{0., 10.}}};

  void SetUp() override {
    TShapeRoadRoutingTest::SetUp();
    start_ = api::RoadPosition(lane_0_0_m1_, api::LanePosition(1., 0., 0.));
    end_ = api::RoadPosition(lane_1_0_m1_, api::LanePosition(10., 0., 0.));
    // Route 0
    route_0_phase_0_start_ = start_;
    route_0_phase_0_end_ = api::RoadPosition(lane_0_0_1_, api::LanePosition(46., 0., 0.));
    route_0_phase_1_start_ = api::RoadPosition(lane_4_0_1_, api::LanePosition(0., 0., 0.));
    route_0_phase_1_end_ = api::RoadPosition(lane_4_0_1_, api::LanePosition(8., 0., 0.));
    route_0_phase_2_start_ = api::RoadPosition(lane_1_0_1_, api::LanePosition(0., 0., 0.));
    route_0_phase_2_end_ = end_;
    // Route 1
    route_1_phase_0_start_ = start_;
    route_1_phase_0_end_ = api::RoadPosition(lane_0_0_m1_, api::LanePosition(46., 0., 0.));
    route_1_phase_1_start_ = api::RoadPosition(lane_5_0_m1_, api::LanePosition(0., 0., 0.));
    route_1_phase_1_end_ = api::RoadPosition(lane_5_0_m1_, api::LanePosition(8., 0., 0.));
    route_1_phase_2_start_ = api::RoadPosition(lane_1_0_m1_, api::LanePosition(0., 0., 0.));
    route_1_phase_2_end_ = end_;
  }

  api::RoadPosition start_;
  api::RoadPosition end_;
  api::RoadPosition route_0_phase_0_start_;
  api::RoadPosition route_0_phase_0_end_;
  api::RoadPosition route_0_phase_1_start_;
  api::RoadPosition route_0_phase_1_end_;
  api::RoadPosition route_0_phase_2_start_;
  api::RoadPosition route_0_phase_2_end_;
  api::RoadPosition route_1_phase_0_start_;
  api::RoadPosition route_1_phase_0_end_;
  api::RoadPosition route_1_phase_1_start_;
  api::RoadPosition route_1_phase_1_end_;
  api::RoadPosition route_1_phase_2_start_;
  api::RoadPosition route_1_phase_2_end_;
};

// No constraints are provided and the only possible route is returned.
TEST_F(DriveForwardStraightOverMultipleLanesTest, WithDefaultConstraintsReturnsRouteWithThreePhases) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kDefaultRoutingConstraints);

  ASSERT_EQ(2u, routes.size());
  {
    const routing::Route& route = routes[0];
    ASSERT_EQ(3, route.size());
    const routing::Phase& phase_0 = route.Get(0);
    const routing::Phase& phase_1 = route.Get(1);
    const routing::Phase& phase_2 = route.Get(2);
    CheckRoutingPhase(phase_0, 0, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_0_start_},
                      std::vector<api::RoadPosition>{route_0_phase_0_end_}, kRoute0Phase0LaneSRanges);
    CheckRoutingPhase(phase_1, 1, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_1_start_},
                      std::vector<api::RoadPosition>{route_0_phase_1_end_}, kRoute0Phase1LaneSRanges);
    CheckRoutingPhase(phase_2, 2, kLinearTolerance, std::vector<api::RoadPosition>{route_0_phase_2_start_},
                      std::vector<api::RoadPosition>{route_0_phase_2_end_}, kRoute0Phase2LaneSRanges);
  }
  {
    const routing::Route& route = routes[1];
    ASSERT_EQ(3, route.size());
    const routing::Phase& phase_0 = route.Get(0);
    const routing::Phase& phase_1 = route.Get(1);
    const routing::Phase& phase_2 = route.Get(2);
    CheckRoutingPhase(phase_0, 0, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_0_start_},
                      std::vector<api::RoadPosition>{route_1_phase_0_end_}, kRoute1Phase0LaneSRanges);
    CheckRoutingPhase(phase_1, 1, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_1_start_},
                      std::vector<api::RoadPosition>{route_1_phase_1_end_}, kRoute1Phase1LaneSRanges);
    CheckRoutingPhase(phase_2, 2, kLinearTolerance, std::vector<api::RoadPosition>{route_1_phase_2_start_},
                      std::vector<api::RoadPosition>{route_1_phase_2_end_}, kRoute1Phase2LaneSRanges);
  }
}

// The maximum cost of the phase is smaller than the solution's phase cost, so no routes can be found.
TEST_F(DriveForwardStraightOverMultipleLanesTest, WithConstrainedPhaseCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallPhaseCostConstraint);

  ASSERT_TRUE(routes.empty());
}

// The maximum cost of the route is smaller than the solution's phase cost, so no routes can be found.
TEST_F(DriveForwardStraightOverMultipleLanesTest, WithConstrainedRouteCostReturnsEmpty) {
  const std::vector<routing::Route> routes = dut_->ComputeRoutes(start_, end_, kSmallRouteCostConstraint);

  ASSERT_TRUE(routes.empty());
}

}  // namespace
}  // namespace test
}  // namespace maliput
