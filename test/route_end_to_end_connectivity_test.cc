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
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/compare.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/common/filesystem.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/routing/phase.h>
#include <maliput/routing/route.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_malidrive/loader/loader.h>

#include "assert_compare.h"

// @file This file evaluates the end to end connectivity constraints of a maliput::routing::Route.
// @details This is not done in maliput because of the need of a concrete maliput::api::RoadGeometry
// to evaluate all the topological and geometrical constraints. Mocking that for the purposes of
// this test would require implementing a concrete backend. In this test, an ad-hoc xodr map is used.
// We don't need any fancy geometry or network topology, just two roads with two lanes each are enough
// to validate the end to end connectivity is correct.

namespace maliput {
namespace test {
namespace {

// Converts @p errors into a single string by concatenating each item with a " | " as separation.
// @param errors A vector of strings.
// @return A string with all items in @p error concatenated with " | " as separation.
std::string SerializeErrors(const std::vector<std::string>& errors) {
  std::string result;
  for (const std::string& error : errors) {
    result += error + " | ";
  }
  return result;
}

class MultipleStraightSegmentsTest : public ::testing::Test {
 public:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{1.0};
  const std::string kXodrFileName{"MultipleStraightSegments.xodr"};
  //@}

  static constexpr double kLaneLength{100.};
  const api::LaneId kLane_1_0_1{"1_0_1"};
  const api::LaneId kLane_1_0_m1{"1_0_-1"};
  const api::LaneId kLane_2_0_1{"2_0_1"};
  const api::LaneId kLane_2_0_m1{"2_0_-1"};
  const api::Lane* lane_1_0_1;
  const api::Lane* lane_1_0_m1;
  const api::Lane* lane_2_0_1;
  const api::Lane* lane_2_0_m1;

  std::unique_ptr<api::RoadNetwork> road_network_{};

  void SetUp() override {
    std::map<std::string, std::string> road_network_configuration;
    road_network_configuration.emplace(malidrive::builder::params::kRoadGeometryId, "malidrive_rg");
    road_network_configuration.emplace(malidrive::builder::params::kOpendriveFile, kXodrFileName);
    road_network_configuration.emplace(malidrive::builder::params::kLinearTolerance, std::to_string(kLinearTolerance));
    road_network_configuration.emplace(malidrive::builder::params::kAngularTolerance,
                                       std::to_string(kAngularTolerance));
    road_network_configuration.emplace(malidrive::builder::params::kScaleLength, std::to_string(kScaleLength));
    road_network_configuration.emplace(malidrive::builder::params::kInertialToBackendFrameTranslation, "{0., 0., 0.}");
    road_network_ = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(road_network_configuration);
    lane_1_0_1 = road_network_->road_geometry()->ById().GetLane(kLane_1_0_1);
    lane_1_0_m1 = road_network_->road_geometry()->ById().GetLane(kLane_1_0_m1);
    lane_2_0_1 = road_network_->road_geometry()->ById().GetLane(kLane_2_0_1);
    lane_2_0_m1 = road_network_->road_geometry()->ById().GetLane(kLane_2_0_m1);

    ASSERT_NE(nullptr, lane_1_0_1);
    ASSERT_NE(nullptr, lane_1_0_m1);
    ASSERT_NE(nullptr, lane_2_0_1);
    ASSERT_NE(nullptr, lane_2_0_m1);
  }
};

class SinglePhaseTest : public MultipleStraightSegmentsTest {};

TEST_F(SinglePhaseTest, SinglePhaseWithOneLaneIsConnectedEndToEnd) {
  const api::RoadPosition start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength));
  const routing::Phase phase(0 /*index*/, kLinearTolerance, {start_position}, {end_position}, {lane_s_range},
                             road_network_.get());
  const routing::Route dut({phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_TRUE(errors.empty()) << SerializeErrors(errors);
}

TEST_F(SinglePhaseTest, SinglePhaseWithTwoLanesIsConnectedEndToEnd) {
  const api::RoadPosition start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange lane_s_range_1(kLane_1_0_1, api::SRange(0, kLaneLength));
  const api::LaneSRange lane_s_range_m1(kLane_1_0_m1, api::SRange(0, kLaneLength));
  const routing::Phase phase(0 /*index*/, kLinearTolerance, {start_position}, {end_position},
                             {lane_s_range_m1, lane_s_range_1}, road_network_.get());
  const routing::Route dut({phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_TRUE(errors.empty()) << SerializeErrors(errors);
}

TEST_F(SinglePhaseTest, SinglePhaseWithTwoLanesAndStartPositionsIsNotConnectedEndToEnd) {
  const api::RoadPosition start_position_1{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition start_position_m1{lane_1_0_m1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange lane_s_range_1(kLane_1_0_1, api::SRange(0, kLaneLength));
  const api::LaneSRange lane_s_range_m1(kLane_1_0_m1, api::SRange(0, kLaneLength));
  const routing::Phase phase(0 /*index*/, kLinearTolerance, {start_position_1, start_position_m1}, {end_position},
                             {lane_s_range_m1, lane_s_range_1}, road_network_.get());
  const routing::Route dut({phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

TEST_F(SinglePhaseTest, SinglePhaseWithTwoLanesAndEndPositionsIsNotConnectedEndToEnd) {
  const api::RoadPosition start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition end_position_1{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::RoadPosition end_position_m1{lane_1_0_m1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange lane_s_range_1(kLane_1_0_1, api::SRange(0, kLaneLength));
  const api::LaneSRange lane_s_range_m1(kLane_1_0_m1, api::SRange(0, kLaneLength));
  const routing::Phase phase(0 /*index*/, kLinearTolerance, {start_position}, {end_position_1, end_position_m1},
                             {lane_s_range_m1, lane_s_range_1}, road_network_.get());
  const routing::Route dut({phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

class MultiPhaseTest : public MultipleStraightSegmentsTest {};

TEST_F(MultiPhaseTest, MultiPhaseWithOneLaneEachIsConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position{lane_2_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition last_end_position{lane_2_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range(kLane_2_0_1, api::SRange(0, kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position}, {last_end_position},
                                  {last_lane_s_range}, road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_TRUE(errors.empty()) << SerializeErrors(errors);
}

TEST_F(MultiPhaseTest, MultiPhaseWithOneLaneEachIsNotGeometricallyConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position{lane_2_0_1, api::LanePosition{10., 0., 0.}};
  const api::RoadPosition last_end_position{lane_2_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range(kLane_2_0_1, api::SRange(10., kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position}, {last_end_position},
                                  {last_lane_s_range}, road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

TEST_F(MultiPhaseTest, MultiPhaseWithOneLaneEachIsNotTopologicallyConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position{lane_2_0_m1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition last_end_position{lane_2_0_m1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range(kLane_2_0_m1, api::SRange(0., kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position}, {last_end_position},
                                  {last_lane_s_range}, road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

TEST_F(MultiPhaseTest, MultiPhaseWithDifferrentNumberOfPositionsAtInterfaceIsNotConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position_1{lane_2_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition last_start_position_m1{lane_2_0_m1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition last_end_position{lane_2_0_m1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range_1(kLane_2_0_1, api::SRange(0., kLaneLength));
  const api::LaneSRange last_lane_s_range_m1(kLane_2_0_m1, api::SRange(0., kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position_1, last_start_position_m1},
                                  {last_end_position}, {last_lane_s_range_1, last_lane_s_range_m1},
                                  road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

TEST_F(MultiPhaseTest, MultiPhaseWithinSameLaneIsConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength / 2., 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength / 2.));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position{lane_1_0_1, api::LanePosition{kLaneLength / 2., 0., 0.}};
  const api::RoadPosition last_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range(kLane_1_0_1, api::SRange(kLaneLength / 2., kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position}, {last_end_position},
                                  {last_lane_s_range}, road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_TRUE(errors.empty()) << SerializeErrors(errors);
}

TEST_F(MultiPhaseTest, MultiPhaseWithinSameLaneIsNotGeometricallyConnectedEndToEnd) {
  const api::RoadPosition first_start_position{lane_1_0_1, api::LanePosition{0., 0., 0.}};
  const api::RoadPosition first_end_position{lane_1_0_1, api::LanePosition{kLaneLength / 2., 0., 0.}};
  const api::LaneSRange first_lane_s_range(kLane_1_0_1, api::SRange(0, kLaneLength / 2.));
  const routing::Phase first_phase(0 /*index*/, kLinearTolerance, {first_start_position}, {first_end_position},
                                   {first_lane_s_range}, road_network_.get());
  const api::RoadPosition last_start_position{lane_1_0_1, api::LanePosition{kLaneLength / 2. + 5., 0., 0.}};
  const api::RoadPosition last_end_position{lane_1_0_1, api::LanePosition{kLaneLength, 0., 0.}};
  const api::LaneSRange last_lane_s_range(kLane_1_0_1, api::SRange(kLaneLength / 2. + 5., kLaneLength));
  const routing::Phase last_phase(1 /*index*/, kLinearTolerance, {last_start_position}, {last_end_position},
                                  {last_lane_s_range}, road_network_.get());
  const routing::Route dut({first_phase, last_phase}, road_network_.get());

  const std::vector<std::string> errors = dut.ValidateEndToEndConnectivity();

  ASSERT_FALSE(errors.empty());
}

}  // namespace
}  // namespace test
}  // namespace maliput
