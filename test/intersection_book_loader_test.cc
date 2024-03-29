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
#include "maliput/base/intersection_book_loader.h"

#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/rules/phase.h>
#include <maliput/api/rules/phase_ring.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/traffic_light_book.h>
#include <maliput/base/manual_phase_provider.h>
#include <maliput/base/phase_ring_book_loader.h>
#include <maliput/base/road_rulebook_loader.h>
#include <maliput/base/traffic_light_book_loader.h>
#include <maliput/common/filesystem.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>

namespace maliput {
namespace {

using api::Intersection;
using api::IntersectionBook;
using api::RoadGeometry;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RoadRulebook;
using api::rules::TrafficLightBook;

static constexpr char kMultilaneResourcesPath[] = DEF_MULTILANE_RESOURCES;

class TestLoading2x2IntersectionIntersectionBook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionIntersectionBook()
      : filepath_(std::string(kMultilaneResourcesPath) + "/2x2_intersection.yaml"),
        road_geometry_(multilane::LoadFile(multilane::BuilderFactory(), filepath_)),
        rulebook_(LoadRoadRulebookFromFile(road_geometry_.get(), filepath_)),
        traffic_light_book_(LoadTrafficLightBookFromFile(filepath_)),
        ring_book_(LoadPhaseRingBookFromFileOldRules(rulebook_.get(), traffic_light_book_.get(), filepath_)) {}

  const std::string filepath_;
  const std::unique_ptr<const RoadGeometry> road_geometry_;
  const std::unique_ptr<const RoadRulebook> rulebook_;
  const std::unique_ptr<const TrafficLightBook> traffic_light_book_;
  const std::unique_ptr<const PhaseRingBook> ring_book_;
};

TEST_F(TestLoading2x2IntersectionIntersectionBook, LoadFromFile) {
  const PhaseRing::Id ring_id("2x2Intersection");
  const std::optional<PhaseRing> ring = ring_book_->GetPhaseRing(PhaseRing::Id(ring_id));
  EXPECT_TRUE(ring.has_value());

  ManualPhaseProvider phase_provider;
  std::unique_ptr<api::IntersectionBook> book =
      LoadIntersectionBookFromFile(filepath_, *rulebook_, *ring_book_, road_geometry_.get(), &phase_provider);
  EXPECT_NE(book, nullptr);
  EXPECT_EQ(int(book->GetIntersections().size()), 1);
  EXPECT_EQ(book->GetIntersection(Intersection::Id("unknown")), nullptr);
  Intersection* intersection = book->GetIntersection(Intersection::Id("2x2Intersection"));
  EXPECT_NE(intersection, nullptr);
  EXPECT_TRUE(intersection->Phase().has_value());
  EXPECT_EQ(intersection->Phase()->state, Phase::Id("NorthSouthPhase"));
  EXPECT_TRUE(intersection->Phase()->next.has_value());
  EXPECT_GT(intersection->region().size(), 0u);
  EXPECT_EQ(intersection->ring_id(), PhaseRing::Id("2x2Intersection"));
}

}  // namespace
}  // namespace maliput
