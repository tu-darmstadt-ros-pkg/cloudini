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
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/pcl_conversion.hpp"
#include "data_path.hpp"

using namespace Cloudini;

TEST(Cloudini, PCD_Encode) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const std::string filepath = Cloudini::tests::DATA_PATH + "lidar.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, cloud) == -1) {
    throw std::runtime_error(std::string("Failed to load PCD file:") + filepath);
  }

  const float resolution = 0.001f;

  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZI);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;
  info.fields.push_back({"x", 0, FieldType::FLOAT32, resolution});
  info.fields.push_back({"y", 4, FieldType::FLOAT32, resolution});
  info.fields.push_back({"z", 8, FieldType::FLOAT32, resolution});
  info.fields.push_back({"intensity", 16, FieldType::FLOAT32, resolution});

  std::vector<uint8_t> compressed_data;

  {
    PointcloudEncoder encoder(info);
    ConstBufferView cloud_points_view(cloud.points.data(), cloud.points.size() * sizeof(pcl::PointXYZI));
    encoder.encode(cloud_points_view, compressed_data);

    auto ratio = static_cast<float>(compressed_data.size()) / static_cast<float>(cloud_points_view.size());
    std::cout << "Compressed data size: " << compressed_data.size() << " percent:" << 100.0 * ratio << std::endl;
  }

  ConstBufferView compressed_view(compressed_data.data(), compressed_data.size());
  auto info_decoded = DecodeHeader(compressed_view);

  ASSERT_EQ(info_decoded.width, info.width);
  ASSERT_EQ(info_decoded.height, info.height);
  ASSERT_EQ(info_decoded.point_step, info.point_step);
  ASSERT_EQ(info_decoded.encoding_opt, info.encoding_opt);
  ASSERT_EQ(info_decoded.compression_opt, info.compression_opt);
  ASSERT_EQ(info_decoded.fields.size(), info.fields.size());
  for (size_t i = 0; i < info.fields.size(); ++i) {
    ASSERT_EQ(info_decoded.fields[i].name, info.fields[i].name);
    ASSERT_EQ(info_decoded.fields[i].offset, info.fields[i].offset);
    ASSERT_EQ(info_decoded.fields[i].type, info.fields[i].type);
    ASSERT_EQ(info_decoded.fields[i].resolution, info.fields[i].resolution);
  }

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  cloud_out.resize(info_decoded.width * info_decoded.height);

  {
    PointcloudDecoder decoder;
    BufferView cloud_points_view(cloud_out.points.data(), cloud_out.points.size() * sizeof(pcl::PointXYZI));
    decoder.decode(info_decoded, compressed_view, cloud_points_view);
  }

  ASSERT_EQ(cloud.points.size(), cloud_out.points.size());

  auto tolerance = resolution * 1.01F;

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].intensity, cloud_out.points[i].intensity, tolerance) << "i:" << i;
  }
}

// pcl::PointXYZRGB stores the color in a FLOAT32 field named "rgb" whose bit pattern is a packed
// integer. Lossy encoding must leave it untouched instead of quantizing it to zero.
TEST(Cloudini, PCL_PackedColorSurvivesLossyRoundtrip) {
  constexpr float kResolution = 0.001F;
  constexpr size_t kPointCount = 256;

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.resize(kPointCount);
  cloud.width = kPointCount;
  cloud.height = 1;
  for (size_t i = 0; i < kPointCount; ++i) {
    auto& point = cloud.points[i];
    point.x = 0.01F * static_cast<float>(i);
    point.y = -0.02F * static_cast<float>(i);
    point.z = 0.5F + 0.03F * static_cast<float>(i);
    point.r = static_cast<uint8_t>(i);
    point.g = static_cast<uint8_t>(255 - i);
    point.b = static_cast<uint8_t>(i * 7);
  }

  pcl::PCLPointCloud2 cloud_msg;
  pcl::toPCLPointCloud2(cloud, cloud_msg);

  std::vector<uint8_t> serialized_data;
  PCLPointCloudEncode(cloud_msg, serialized_data, kResolution);

  pcl::PCLPointCloud2 decoded_msg;
  PCLPointCloudDecode(ConstBufferView(serialized_data.data(), serialized_data.size()), decoded_msg);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
  pcl::fromPCLPointCloud2(decoded_msg, cloud_out);

  ASSERT_EQ(cloud_out.points.size(), cloud.points.size());

  const auto tolerance = kResolution * 1.01F;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
    ASSERT_EQ(cloud.points[i].r, cloud_out.points[i].r) << "i:" << i;
    ASSERT_EQ(cloud.points[i].g, cloud_out.points[i].g) << "i:" << i;
    ASSERT_EQ(cloud.points[i].b, cloud_out.points[i].b) << "i:" << i;
  }
}

TEST(Cloudini, PCL_Conversion) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const std::string filepath = Cloudini::tests::DATA_PATH + "lidar.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, cloud) == -1) {
    throw std::runtime_error(std::string("Failed to load PCD file:") + filepath);
  }

  const float resolution = 0.001f;

  std::vector<uint8_t> serialized_data;
  PointcloudEncode(cloud, serialized_data, 0.001);

  const auto tolerance = resolution * 1.01F;

  {
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    PointcloudDecode(serialized_data, cloud_out);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].intensity, cloud_out.points[i].intensity, tolerance) << "i:" << i;
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    PointcloudDecode(serialized_data, cloud_out);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
    }
  }
}
