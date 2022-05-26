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
#include "maliput/base/traffic_light_book_loader.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/api/rules/traffic_lights.h>
#include <maliput/common/filesystem.h>
#include <maliput/math/quaternion.h>
#include <maliput/test_utilities/rules_test_utilities.h>
#include <maliput/test_utilities/traffic_lights_compare.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>

namespace maliput {
namespace {

using api::InertialPosition;
using api::Rotation;
using api::rules::Bulb;
using api::rules::BulbColor;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::BulbType;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbId;
constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

class TestLoading2x2IntersectionTrafficLightbook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionTrafficLightbook()
      : filepath_(maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml") {
    std::vector<std::unique_ptr<Bulb>> bulbs;
    std::vector<std::unique_ptr<BulbGroup>> bulb_groups;

    const Rotation kRotation = Rotation::FromQuat(math::Quaternion(1, 0, 0, 0));

    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("RedBulb"), InertialPosition(0, 0, 0.3937), kRotation, BulbColor::kRed, BulbType::kRound, std::nullopt,
        std::vector<BulbState>({BulbState::kOn, BulbState::kOff}), Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("YellowBulb"), InertialPosition(0, 0, 0), kRotation, BulbColor::kYellow, BulbType::kRound,
        std::nullopt, std::vector<BulbState>({BulbState::kOn, BulbState::kOff}), Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("GreenBulb"), InertialPosition(0, 0, -0.3937), kRotation, BulbColor::kGreen, BulbType::kRound,
        std::nullopt, std::vector<BulbState>({BulbState::kOn, BulbState::kOff}), Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("YellowLeftArrowBulb"), InertialPosition(0, -0.3937, -0.3937), kRotation, BulbColor::kYellow,
        BulbType::kArrow, 3.14, std::vector<BulbState>({BulbState::kOff, BulbState::kBlinking}), Bulb::BoundingBox()));
    bulb_groups.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("SouthFacingBulbs"), InertialPosition(0, 0, 0),
                                                      kRotation, std::move(bulbs)));
    south_facing_ = std::make_unique<const TrafficLight>(
        TrafficLight::Id("SouthFacing"), InertialPosition(1.875, 9.375, 6.0579),
        Rotation::FromQuat(math::Quaternion(0.707107, 0, 0, -0.707107)), std::move(bulb_groups));
  }

  const std::string filepath_;
  std::unique_ptr<const TrafficLight> south_facing_;
};

// Fully check the south-facing traffic light. Then spot-check the rest of them.
TEST_F(TestLoading2x2IntersectionTrafficLightbook, LoadFromFile) {
  std::unique_ptr<TrafficLightBook> book = LoadTrafficLightBookFromFile(filepath_);
  EXPECT_NE(book, nullptr);
  const TrafficLight* south_facing = book->GetTrafficLight(TrafficLight::Id("SouthFacing"));
  EXPECT_NE(south_facing, nullptr);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(south_facing, south_facing_.get()));
  for (const auto& name : {"NorthFacing", "EastFacing", "WestFacing"}) {
    EXPECT_NE(book->GetTrafficLight(TrafficLight::Id(name)), nullptr);
  }
}

}  // namespace
}  // namespace maliput
