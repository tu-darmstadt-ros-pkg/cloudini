/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <iostream>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/encoding_utils.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "data_path.hpp"

using namespace Cloudini;

template <typename TypeA, typename TypeB>
void CompareInfos(const TypeA& a, const TypeB& b) {
  ASSERT_EQ(a.width, b.width);
  ASSERT_EQ(a.height, b.height);
  ASSERT_EQ(a.point_step, b.point_step);
  ASSERT_EQ(a.point_step, b.point_step);
  ASSERT_EQ(a.fields.size(), b.fields.size());

  for (size_t i = 0; i < a.fields.size(); ++i) {
    ASSERT_EQ(a.fields[i].name, b.fields[i].name);
    ASSERT_EQ(a.fields[i].type, b.fields[i].type);
    ASSERT_EQ(a.fields[i].offset, b.fields[i].offset);
  }
}

void VerifyRoundTrip(const EncodingInfo& encoding_info, const std::vector<uint8_t>& original_data, float resolution) {
  // Encode the point cloud data

  PointcloudEncoder pc_encoder(encoding_info);
  std::vector<uint8_t> compressed_data;
  pc_encoder.encode(original_data, compressed_data);

  // Decode the point cloud data
  PointcloudDecoder pc_decoder;
  std::vector<uint8_t> decoded_data;
  ConstBufferView compressed_view(compressed_data.data(), compressed_data.size());
  auto recovered_header = DecodeHeader(compressed_view);
  pc_decoder.decode(recovered_header, compressed_view, decoded_data);

  CompareInfos(encoding_info, recovered_header);
  const uint8_t* original_data_ptr = original_data.data();
  const uint8_t* decoded_data_ptr = decoded_data.data();
  int offset = 0;

  const auto& fields = encoding_info.fields;

  for (size_t i = 0; i < encoding_info.width * encoding_info.height; ++i) {
    float original_x = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[0].offset);
    float original_y = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[1].offset);
    float original_z = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[2].offset);
    float original_intensity = *reinterpret_cast<const float*>(original_data_ptr + offset + fields[3].offset);
    uint16_t original_ring = *reinterpret_cast<const uint16_t*>(original_data_ptr + offset + fields[4].offset);
    double original_timestamp = *reinterpret_cast<const double*>(original_data_ptr + offset + fields[5].offset);

    float decoded_x = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[0].offset);
    float decoded_y = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[1].offset);
    float decoded_z = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[2].offset);
    float decoded_intensity = *reinterpret_cast<const float*>(decoded_data_ptr + offset + fields[3].offset);
    uint16_t decoded_ring = *reinterpret_cast<const uint16_t*>(decoded_data_ptr + offset + fields[4].offset);
    double decoded_timestamp = *reinterpret_cast<const double*>(decoded_data_ptr + offset + fields[5].offset);

    ASSERT_NEAR(original_x, decoded_x, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_y, decoded_y, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_z, decoded_z, resolution) << "Point index: " << i;
    ASSERT_NEAR(original_intensity, decoded_intensity, resolution) << "Point index: " << i;
    ASSERT_EQ(original_ring, decoded_ring) << "Point index: " << i;
    ASSERT_EQ(original_timestamp, decoded_timestamp) << "Point index: " << i;

    offset += recovered_header.point_step;
  }
}

TEST(Cloudini, DDS_Roundtrip) {
  const std::string filepath = Cloudini::tests::DATA_PATH + "dds_message.bin";

  std::vector<uint8_t> dds_pointcloud_msg;
  {
    std::ifstream file(filepath, std::ios::binary);
    ASSERT_TRUE(file.is_open()) << "Failed to open file: " << filepath;

    file.seekg(0, std::ios::end);
    dds_pointcloud_msg.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(dds_pointcloud_msg.data()), dds_pointcloud_msg.size());
    file.close();
  }

  const float resolution = 0.001f;

  using namespace Cloudini;

  const EncodingInfo expected_infos = {
      .fields =
          {
              {"x", 0, FieldType::FLOAT32, resolution},
              {"y", 4, FieldType::FLOAT32, resolution},
              {"z", 8, FieldType::FLOAT32, resolution},
              {"intensity", 12, FieldType::FLOAT32, resolution},
              {"ring", 16, FieldType::UINT16},
              {"timestamp", 18, FieldType::FLOAT64},
          },
      .width = 64000,
      .height = 1,
      .point_step = 26,
      .encoding_opt = EncodingOptions::LOSSY,
      .compression_opt = CompressionOption::ZSTD,
      .version = kEncodingVersion};

  //-----------------------------------------------------------------------------
  // read the DDS message
  const auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(dds_pointcloud_msg);
  EncodingInfo encoding_info = cloudini_ros::toEncodingInfo(pc_info);

  CompareInfos(pc_info, expected_infos);
  CompareInfos(encoding_info, expected_infos);

  encoding_info.fields[0].resolution = resolution;  // Set resolution for x
  encoding_info.fields[1].resolution = resolution;  // Set resolution for y
  encoding_info.fields[2].resolution = resolution;  // Set resolution for z
  encoding_info.fields[3].resolution = resolution;  // Set resolution for intensity

  //-----------------------------------------------------------------------------
  const std::vector<uint8_t> original_data(pc_info.data.data(), pc_info.data.data() + pc_info.data.size());

  VerifyRoundTrip(encoding_info, original_data, resolution);
}

// Layout of a pcl::PointXYZRGB serialized as sensor_msgs/PointCloud2:
// x,y,z are real-valued FLOAT32, "rgb" is a FLOAT32 whose *bit pattern* is a
// packed 0x00RRGGBB integer.
struct PointXYZRGB_Layout {
  float x;
  float y;
  float z;
  float padding;
  float rgb;
  float padding2[3];
};

static float packRGB(uint8_t r, uint8_t g, uint8_t b) {
  const uint32_t packed = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b;
  float as_float = 0.0F;
  std::memcpy(&as_float, &packed, sizeof(packed));
  return as_float;
}

static uint32_t unpackRGB(float value) {
  uint32_t packed = 0;
  std::memcpy(&packed, &value, sizeof(packed));
  return packed & 0x00FFFFFFU;
}

TEST(Cloudini, PackedColorFieldSurvivesLossyRoundtrip) {
  constexpr float kResolution = 0.001F;
  constexpr size_t kPointCount = 64;

  std::vector<Cloudini::PointField> fields = {
      {"x", offsetof(PointXYZRGB_Layout, x), FieldType::FLOAT32, std::nullopt},
      {"y", offsetof(PointXYZRGB_Layout, y), FieldType::FLOAT32, std::nullopt},
      {"z", offsetof(PointXYZRGB_Layout, z), FieldType::FLOAT32, std::nullopt},
      {"rgb", offsetof(PointXYZRGB_Layout, rgb), FieldType::FLOAT32, std::nullopt},
  };
  cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, fields, kResolution);

  // The packed color field must not be assigned a quantization resolution.
  ASSERT_TRUE(fields[0].resolution.has_value());
  ASSERT_FALSE(fields[3].resolution.has_value());

  EncodingInfo info;
  info.fields = fields;
  info.width = kPointCount;
  info.height = 1;
  info.point_step = sizeof(PointXYZRGB_Layout);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;

  std::vector<PointXYZRGB_Layout> points(kPointCount, PointXYZRGB_Layout{});
  for (size_t i = 0; i < kPointCount; ++i) {
    points[i].x = 0.01F * static_cast<float>(i);
    points[i].y = 0.02F * static_cast<float>(i);
    points[i].z = 0.03F * static_cast<float>(i);
    points[i].rgb = packRGB(static_cast<uint8_t>(i * 3), static_cast<uint8_t>(255 - i), static_cast<uint8_t>(i ^ 0x5A));
  }

  const std::vector<uint8_t> original_data(
      reinterpret_cast<const uint8_t*>(points.data()),
      reinterpret_cast<const uint8_t*>(points.data()) + points.size() * sizeof(PointXYZRGB_Layout));

  PointcloudEncoder pc_encoder(info);
  std::vector<uint8_t> compressed_data;
  pc_encoder.encode(original_data, compressed_data);

  PointcloudDecoder pc_decoder;
  std::vector<uint8_t> decoded_data;
  ConstBufferView compressed_view(compressed_data.data(), compressed_data.size());
  auto recovered_header = DecodeHeader(compressed_view);
  pc_decoder.decode(recovered_header, compressed_view, decoded_data);

  ASSERT_EQ(decoded_data.size(), original_data.size());
  const auto* decoded = reinterpret_cast<const PointXYZRGB_Layout*>(decoded_data.data());

  for (size_t i = 0; i < kPointCount; ++i) {
    EXPECT_NEAR(points[i].x, decoded[i].x, kResolution) << "Point index: " << i;
    EXPECT_NEAR(points[i].y, decoded[i].y, kResolution) << "Point index: " << i;
    EXPECT_NEAR(points[i].z, decoded[i].z, kResolution) << "Point index: " << i;
    EXPECT_EQ(unpackRGB(points[i].rgb), unpackRGB(decoded[i].rgb)) << "Point index: " << i;
  }
}

TEST(Cloudini, RosPointCloud2CopyRebindsOwnedDataView) {
  cloudini_ros::RosPointCloud2 original;
  original.owned_data = {1, 2, 3, 4};
  original.data = ConstBufferView(original.owned_data.data(), original.owned_data.size());

  const cloudini_ros::RosPointCloud2 copied = original;

  ASSERT_EQ(copied.owned_data, original.owned_data);
  EXPECT_EQ(copied.data.data(), copied.owned_data.data());
  EXPECT_EQ(copied.data.size(), copied.owned_data.size());
}
