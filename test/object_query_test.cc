#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/common/filesystem.h>
#include <maliput_malidrive/builder/params.h>
#include <maliput_malidrive/builder/road_network_builder.h>
#include <maliput_multilane/builder.h>
#include <maliput_multilane/loader.h>
#include <maliput_object/api/object_query.h>
#include <maliput_object/base/bounding_box.h>
#include <maliput_object/base/simple_object_query.h>
#include <maliput_object/test_utilities/mock.h>

namespace maliput {
namespace {

#define EXPECT_FOUND_IN(item, expected_items)                                                       \
  do {                                                                                              \
    EXPECT_NE(expected_items.end(), std::find(expected_items.begin(), expected_items.end(), item)); \
  } while (0)

using api::RoadNetwork;

constexpr char MALIPUT_MALIDRIVE_RESOURCE_VAR[] = "MALIPUT_MALIDRIVE_RESOURCE_ROOT";

struct SimpleObjectQueryResults {
  maliput::math::Vector3 box_position{};
  maliput::math::Vector3 box_size{};
  maliput::math::RollPitchYaw box_orientation{};
  std::vector<maliput::api::LaneId> expected_lane_ids;
};

std::vector<SimpleObjectQueryResults> InstantiateSimpleObjectQueryParameters() {
  return {
      // At the middle of the lane 0_0_1.
      {{25., 1.75, 0.}, {0.5, 0.5, 2.}, {0., 0., 0.}, {maliput::api::LaneId{"0_0_1"}}},
      // At the start of the segment 0_0.
      {{0., 0., 0.}, {5., 5., 2.}, {0., 0., 0.}, {maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"0_0_1"}}},
      // At the middle of the T intersection.
      {{50., 0, 0.},
       {0.5, 0.5, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"},
        maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}}},
      // At the middle of the T intersection. Extruded in y direction.
      {{50., 0, 0.},
       {0.5, 10., 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"}, maliput::api::LaneId{"4_0_1"},
        maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"},
        maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}}},
      // At the middle of the T intersection. Extruded in x direction.
      {{50., 0, 0.},
       {10, 0.5, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"},
        maliput::api::LaneId{"6_0_-1"}, maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"},
        maliput::api::LaneId{"9_0_-1"}}},
      // At the middle of the T intersection. Extruded in both x and y direction. All lanes are intersected.
      {{50., 0, 0.},
       {10, 10, 2.},
       {0., 0., 0.},
       {maliput::api::LaneId{"0_0_1"}, maliput::api::LaneId{"0_0_-1"}, maliput::api::LaneId{"1_0_1"},
        maliput::api::LaneId{"1_0_-1"}, maliput::api::LaneId{"2_0_1"}, maliput::api::LaneId{"2_0_-1"},
        maliput::api::LaneId{"4_0_1"}, maliput::api::LaneId{"5_0_-1"}, maliput::api::LaneId{"6_0_-1"},
        maliput::api::LaneId{"7_0_-1"}, maliput::api::LaneId{"8_0_-1"}, maliput::api::LaneId{"9_0_-1"}}},
  };
}

class TestSimpleObjectQuery : public ::testing::TestWithParam<SimpleObjectQueryResults> {
 public:
  TestSimpleObjectQuery()
      : filepath_(maliput::common::Filesystem::get_env_path(MALIPUT_MALIDRIVE_RESOURCE_VAR) +
                  "/resources/odr/TShapeRoad.xodr") {}

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

TEST_P(TestSimpleObjectQuery, FindOverlappingLanes) {
  const object::api::Object<maliput::math::Vector3> object{
      kId,
      {},
      std::make_unique<object::BoundingBox>(results_.box_position, results_.box_size, results_.box_orientation,
                                            kTolerance)};

  auto lanes = dut_->FindOverlappingLanesIn(&object);
  for (const auto lane : lanes) {
    EXPECT_FOUND_IN(lane->id(), results_.expected_lane_ids);
  }
}

INSTANTIATE_TEST_CASE_P(TestSimpleObjectQueryGroup, TestSimpleObjectQuery,
                        ::testing::ValuesIn(InstantiateSimpleObjectQueryParameters()));

}  // namespace
}  // namespace maliput
