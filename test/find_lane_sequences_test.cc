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
#include "maliput/routing/find_lane_sequences.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/common/filesystem.h>
#include <maliput_dragway_test_utilities/fixtures.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_malidrive/loader/loader.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>
#include <maliput_multilane_test_utilities/fixtures.h>

namespace maliput {
namespace routing {

using maliput::api::Lane;
using maliput::api::LaneId;
using maliput::api::RoadGeometry;
using maliput::dragway::DragwayBasedTest;
using maliput::multilane::BranchAndMergeBasedTest;
using maliput::multilane::LoopBasedTest;
using maliput::multilane::MultiBranchBasedTest;

namespace {

void CheckSequences(const std::vector<std::vector<const Lane*>>& sequences,
                    const std::vector<std::vector<std::string>>& expected_ids) {
  ASSERT_TRUE(sequences.size() == expected_ids.size());
  for (const auto& sequence : sequences) {
    bool found = false;
    for (int j = 0; !found && j < static_cast<int>(expected_ids.size()); ++j) {
      const std::vector<std::string>& expected_seq = expected_ids.at(j);
      bool match = true;
      if (sequence.size() == expected_seq.size()) {
        for (int i = 0; match && i < static_cast<int>(sequence.size()); ++i) {
          if (sequence.at(i)->id().string() != expected_seq.at(i)) {
            match = false;
          }
        }
      } else {
        match = false;
      }
      if (match) {
        found = true;
      }
    }
    ASSERT_TRUE(found);
  }
}

}  // namespace

TEST_F(DragwayBasedTest, FindLaneSequencesChangeLanes) {
  CheckSequences(FindLaneSequences(center_lane_, left_lane_, kLength), {});
  CheckSequences(FindLaneSequences(center_lane_, right_lane_, kLength), {});
  CheckSequences(FindLaneSequences(right_lane_, left_lane_, kLength), {});
  CheckSequences(FindLaneSequences(left_lane_, right_lane_, kLength), {});
}

TEST_F(DragwayBasedTest, FindLaneSequencesChangeLanesWithMaxLength) {
  constexpr double kMaxLength{std::numeric_limits<double>::max()};
  CheckSequences(FindLaneSequences(center_lane_, left_lane_, kMaxLength), {});
  CheckSequences(FindLaneSequences(center_lane_, right_lane_, kMaxLength), {});
  CheckSequences(FindLaneSequences(right_lane_, left_lane_, kMaxLength), {});
  CheckSequences(FindLaneSequences(left_lane_, right_lane_, kMaxLength), {});
}

TEST_F(DragwayBasedTest, FindLaneSequencesSameLane) {
  for (double length : std::vector<double>{kLength, 0.}) {
    CheckSequences(FindLaneSequences(center_lane_, center_lane_, length), {{center_lane_->id().string()}});
    CheckSequences(FindLaneSequences(right_lane_, right_lane_, length), {{right_lane_->id().string()}});
    CheckSequences(FindLaneSequences(left_lane_, left_lane_, length), {{left_lane_->id().string()}});
  }
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartLeftLaneToEndLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_),
                 {{"l:0_1", "l:1.1_0", "l:1.2_0", "l:1.3_0", "l:3_1"}});
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_ / 2), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartRightLaneToEndRightLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_0"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_),
                 {{"l:0_0", "l:2.1_0", "l:2.2_0", "l:2.3_0", "l:3_0"}});
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_ / 2), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartLeftLaneToEndRightLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_1"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_0"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartRightLaneToEndLeftLane) {
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:3_1"));
  CheckSequences(FindLaneSequences(start_lane, end_lane, total_length_), {});
}

TEST_F(BranchAndMergeBasedTest, FindLaneSequencesStartAndStopInSameLane) {
  const std::string lane_id = "l:0_1";
  const Lane* lane = index_.GetLane(LaneId(lane_id));
  for (double length : std::vector<double>{total_length_, total_length_ / 2, 0.}) {
    CheckSequences(FindLaneSequences(lane, lane, length), {{lane_id}});
  }
}

TEST_F(LoopBasedTest, FindLaneSequencesTest) {
  // Arbitrary large number to avoid hitting limit.
  constexpr double kMaxLength{1e6};
  // Verifies the DUT doesn't enter an infinite loop.
  const Lane* start_lane = index_.GetLane(LaneId("l:0_0"));
  const Lane* end_lane = index_.GetLane(LaneId("l:4_0"));
  const std::vector<std::vector<const Lane*>> sequences = FindLaneSequences(start_lane, end_lane, kMaxLength);

  ASSERT_EQ(sequences.size(), 5u);
}

TEST_F(MultiBranchBasedTest, FindLaneSequencesTest) {
  // Arbitrary large number to avoid hitting limit.
  constexpr double kMaxLength{1e6};
  const std::vector<std::pair<const Lane*, const Lane*>> test_cases = {
      std::make_pair(index_.GetLane(LaneId("l:0_0")), index_.GetLane(LaneId("l:1.1_0"))),
      std::make_pair(index_.GetLane(LaneId("l:0_0")), index_.GetLane(LaneId("l:2.1_0"))),
      std::make_pair(index_.GetLane(LaneId("l:0_0")), index_.GetLane(LaneId("l:3.1_0")))};
  for (const auto& test_case : test_cases) {
    CheckSequences(FindLaneSequences(test_case.first, test_case.second, kMaxLength),
                   {{test_case.first->id().string(), test_case.second->id().string()}});
  }
}

static constexpr char kMultilaneResourcesPath[] = DEF_MULTILANE_RESOURCES;

GTEST_TEST(FindLaneSequencesTest, NoRouteToEndLane) {
  std::unique_ptr<const RoadGeometry> road = maliput::multilane::LoadFile(
      maliput::multilane::BuilderFactory(), std::string(kMultilaneResourcesPath) + "/dual_non_intersecting_lanes.yaml");
  const Lane* start_lane = road->junction(0)->segment(0)->lane(0);
  const Lane* end_lane = road->junction(1)->segment(0)->lane(0);
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane, start_lane->length() + end_lane->length()).size(), 0u);
}

GTEST_TEST(FindLaneSequencesTest, MaxLengthOmitsStartAndEndLanes) {
  std::unique_ptr<const RoadGeometry> road = maliput::multilane::LoadFile(
      maliput::multilane::BuilderFactory(), std::string(kMultilaneResourcesPath) + "/long_start_and_end_lanes.yaml");
  const RoadGeometry::IdIndex& index = road->ById();
  const Lane* start_lane = index.GetLane(LaneId("l:0_0"));
  const Lane* middle_lane = index.GetLane(LaneId("l:1_0"));
  const Lane* end_lane = index.GetLane(LaneId("l:2_0"));
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane, middle_lane->length() / 2).size(), 0u);
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane, middle_lane->length()).size(), 1u);
  const double total_length = start_lane->length() + middle_lane->length() + end_lane->length();
  ASSERT_EQ(FindLaneSequences(start_lane, end_lane, total_length).size(), 1u);
}

static constexpr char kMalidriveResourcesPath[] = DEF_MALIDRIVE_RESOURCES;

// Loads TShapeRoad, and evaluates FindLaneSequences() for a large distance that will not
// filter out results, and shows the behavior when removing and leaving U-turns.
class TShapeRoadFindLaneSequencesTest : public ::testing::Test {
 public:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{1.0};
  static constexpr double kDistance{std::numeric_limits<double>::max()};
  static constexpr bool kDontRemoveUTurns{false};
  static constexpr bool kRemoveUTurns{true};
  //@}
  const std::string kTShapeRoadFilePath{std::string(kMalidriveResourcesPath) +
                                        std::string("/resources/odr/TShapeRoad.xodr")};

  const api::LaneId kStartLaneId{"1_0_1"};
  const api::LaneId kEndLaneId{"0_0_1"};
  const Lane* start_lane_;
  const Lane* end_lane_;

  void SetUp() override {
    std::map<std::string, std::string> configuration;
    configuration.emplace(malidrive::builder::params::kRoadGeometryId, "malidrive_rg");
    configuration.emplace(malidrive::builder::params::kOpendriveFile, kTShapeRoadFilePath);
    configuration.emplace(malidrive::builder::params::kLinearTolerance, std::to_string(kLinearTolerance));
    configuration.emplace(malidrive::builder::params::kAngularTolerance, std::to_string(kAngularTolerance));
    configuration.emplace(malidrive::builder::params::kScaleLength, std::to_string(kScaleLength));
    configuration.emplace(malidrive::builder::params::kInertialToBackendFrameTranslation, "{0., 0., 0.}");
    road_network_ = malidrive::loader::Load<malidrive::builder::RoadNetworkBuilder>(configuration);
    start_lane_ = road_network_->road_geometry()->ById().GetLane(kStartLaneId);
    end_lane_ = road_network_->road_geometry()->ById().GetLane(kEndLaneId);
  }

  std::unique_ptr<api::RoadNetwork> road_network_{};
};

TEST_F(TShapeRoadFindLaneSequencesTest, NotRemovingUTurnYieldsTwoSequences) {
  CheckSequences(
      FindLaneSequences(start_lane_, end_lane_, kDistance, kDontRemoveUTurns),
      {{"1_0_1", "6_0_-1", "2_0_1", "9_0_-1", "0_0_-1", "5_0_-1", "1_0_-1", "7_0_-1", "2_0_-1", "8_0_-1", "0_0_1"},
       {"1_0_1", "4_0_1", "0_0_1"}});
}

TEST_F(TShapeRoadFindLaneSequencesTest, RemovingUTurnYieldsOneSequence) {
  CheckSequences(FindLaneSequences(start_lane_, end_lane_, kDistance, kRemoveUTurns), {{"1_0_1", "4_0_1", "0_0_1"}});
}

}  // namespace routing
}  // namespace maliput
