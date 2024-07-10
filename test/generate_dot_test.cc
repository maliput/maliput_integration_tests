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
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/base/distance_router.h>
#include <maliput/routing/graph/graph.h>
#include <maliput/routing/route.h>
#include <maliput/routing/router.h>
#include <maliput/routing/routing_constraints.h>
#include <maliput/utility/generate_dot.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_malidrive/loader/loader.h>

namespace maliput {
namespace test {
namespace {

static constexpr char kMalidriveResourcesPath[] = DEF_MALIDRIVE_RESOURCES;

class TShapeRoadGenerateDotTest : public ::testing::Test {
 public:
  //@{  Tolerances set to match the involved geometries and the parser resolution.
  static constexpr double kLinearTolerance{1e-6};
  static constexpr double kAngularTolerance{1e-6};
  static constexpr double kScaleLength{1.0};
  //@}
  static constexpr routing::RoutingConstraints kDefaultRoutingConstraints{};
  const std::string kTShapeRoadFilePath{std::string(kMalidriveResourcesPath) +
                                        std::string("/resources/odr/TShapeRoad.xodr")};
  const api::LaneId kStartLaneId{"0_0_1"};
  const api::LaneId kEndLaneId{"0_0_1"};

  std::unique_ptr<api::RoadNetwork> road_network_{};
  std::unique_ptr<routing::Router> router_{};
  routing::graph::Graph graph_{};
  std::vector<routing::Route> routes_{};

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

    router_ = std::make_unique<maliput::DistanceRouter>(*road_network_, kLinearTolerance);

    graph_ = routing::graph::BuildGraph(road_network_->road_geometry());

    const api::RoadPosition start(road_network_->road_geometry()->ById().GetLane(kStartLaneId),
                                  api::LanePosition(1., 0., 0.));
    const api::RoadPosition end(road_network_->road_geometry()->ById().GetLane(kEndLaneId),
                                api::LanePosition(10., 0., 0.));
    routes_ = router_->ComputeRoutes(start, end, kDefaultRoutingConstraints);
    ASSERT_EQ(1u, routes_.size());
  }
};

// Evaluates the graph is correct and the api::Segment containing the routing::Route is marked with the red color.
TEST_F(TShapeRoadGenerateDotTest, DotGraphWithARouteMarksInRedWhereTheRouteGoesThrough) {
  const std::string kResult(R"(graph {
1 -- 5 [ label = "9_0" ];
5 -- 1 [ label = "8_0" ];
5 -- 2 [ label = "7_0" ];
1 -- 2 [ label = "5_0" ];
4 -- 5 [ label = "2_0" ];
2 -- 5 [ label = "6_0" ];
1 -- 2 [ label = "4_0" ];
2 -- 3 [ label = "1_0" ];
0 -- 1 [ label = "0_0" color = "red" ];
}
)");

  std::stringstream ss;
  maliput::utility::GenerateDotStream(graph_, routes_.front(), &ss);

  ASSERT_EQ(kResult, ss.str());
}

// Evaluates the graph serialization is the expected one. There is no prescriptive order in the serialization, thus
// differences are expected in the order of the edges.
TEST_F(TShapeRoadGenerateDotTest, DotGraphWithoutARouteYieldsTheExpectedGraph) {
  const std::string kResult(R"(graph {
1 -- 5 [ label = "9_0" ];
5 -- 2 [ label = "7_0" ];
2 -- 5 [ label = "6_0" ];
1 -- 2 [ label = "5_0" ];
5 -- 1 [ label = "8_0" ];
1 -- 2 [ label = "4_0" ];
4 -- 5 [ label = "2_0" ];
2 -- 3 [ label = "1_0" ];
0 -- 1 [ label = "0_0" ];
}
)");

  std::stringstream ss;
  maliput::utility::GenerateDotStream(graph_, &ss);

  ASSERT_EQ(kResult, ss.str());
}

}  // namespace
}  // namespace test
}  // namespace maliput
