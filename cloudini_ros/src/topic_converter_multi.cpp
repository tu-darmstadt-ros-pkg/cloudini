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

#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/*
 * This node converts compressed point cloud messages from the
 * `point_cloud_interfaces/msg/CompressedPointCloud2` format to the
 * `sensor_msgs/msg/PointCloud2` format.
 *
 * It is BRUTALLY efficient, because we read directly the RAW DDS message,
 * and write the output message without any intermediate copies.
 *
 * This means less CPU and less latency.
 *
 * This version supports multiple input/output topic pairs. Author: Jonathan Lichtenfeld
 */
class CloudiniPointcloudConverterMulti : public rclcpp::Node {
 public:
  CloudiniPointcloudConverterMulti(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void callback(size_t topic_index, std::shared_ptr<rclcpp::SerializedMessage> msg);

  ~CloudiniPointcloudConverterMulti() {
    // bypass the deleter for all output messages
    for (auto& output_msg : output_messages_) {
      output_msg.get_rcl_serialized_message().buffer = nullptr;
      output_msg.get_rcl_serialized_message().buffer_length = 0;
    }
  }

 private:
  // generic subscribers for compressed point cloud messages
  std::vector<rclcpp::GenericSubscription::SharedPtr> point_cloud_subscribers_;

  // generic publishers for sensor_msgs/msg/PointCloud2 (but... raw DDS message)
  std::vector<rclcpp::GenericPublisher::SharedPtr> point_cloud_publishers_;

  // Per-topic buffers
  std::vector<std::vector<uint8_t>> output_raw_messages_;
  std::vector<rclcpp::SerializedMessage> output_messages_;

  bool compressing_ = true;
  double resolution_ = 0.001;  // 1mm

  // Per-topic statistics
  std::vector<uint64_t> tot_original_sizes_;
  std::vector<uint64_t> tot_compressed_sizes_;
  std::vector<int> message_counts_;
};
//-----------------------------------------------------

rclcpp::QoS adapt_request_to_offers(
    const std::string& topic_name, const std::vector<rclcpp::TopicEndpointInfo>& endpoints) {
  rclcpp::QoS request_qos(rmw_qos_profile_default.depth);

  if (endpoints.empty()) {
    return request_qos;
  }
  size_t reliability_reliable_endpoints_count = 0;
  size_t durability_transient_local_endpoints_count = 0;
  for (const auto& endpoint : endpoints) {
    const auto& profile = endpoint.qos_profile().get_rmw_qos_profile();
    if (profile.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
      reliability_reliable_endpoints_count++;
    }
    if (profile.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
      durability_transient_local_endpoints_count++;
    }
  }
  // Policy: reliability
  if (reliability_reliable_endpoints_count == endpoints.size()) {
    request_qos.reliable();
  } else {
    request_qos.best_effort();
  }
  // Policy: durability
  // If all publishers offer transient_local, we can request it and receive latched messages
  if (durability_transient_local_endpoints_count == endpoints.size()) {
    request_qos.transient_local();
  } else {
    request_qos.durability_volatile();
  }
  return request_qos;
}
//-----------------------------------------------------

CloudiniPointcloudConverterMulti::CloudiniPointcloudConverterMulti(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cloudini_pointcloud_converter_multi", options) {
  // Declare parameters for input and output topics
  this->declare_parameter<bool>("compressing", true);
  this->declare_parameter<std::vector<std::string>>("topics_input", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("topics_output", std::vector<std::string>{});
  this->declare_parameter<double>("resolution", 0.001);

  // read parameters
  compressing_ = this->get_parameter("compressing").as_bool();
  resolution_ = this->get_parameter("resolution").as_double();

  const std::vector<std::string> input_topics = this->get_parameter("topics_input").as_string_array();
  if (input_topics.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input topics list is empty or not set");
    throw std::runtime_error("Input topics list is required");
  }

  std::vector<std::string> output_topics = this->get_parameter("topics_output").as_string_array();

  // If output topics not provided, generate them
  if (output_topics.empty()) {
    output_topics.reserve(input_topics.size());
    const std::string suffix = compressing_ ? "/compressed" : "/decompressed";
    for (const auto& input_topic : input_topics) {
      output_topics.push_back(input_topic + suffix);
    }
    RCLCPP_WARN(
        this->get_logger(), "Output topics not set, using auto-generated names with suffix '%s'", suffix.c_str());
  } else {
    // Validate that output topics match input topics in size
    if (output_topics.size() != input_topics.size()) {
      RCLCPP_ERROR(
          this->get_logger(), "Output topics size (%zu) does not match input topics size (%zu)", output_topics.size(),
          input_topics.size());
      throw std::runtime_error("Output topics size must match input topics size");
    }
  }

  const std::string compressed_topic_type = "point_cloud_interfaces/msg/CompressedPointCloud2";
  const std::string pointcloud_topic_type = "sensor_msgs/msg/PointCloud2";

  const std::string input_topic_type = compressing_ ? pointcloud_topic_type : compressed_topic_type;
  const std::string output_topic_type = compressing_ ? compressed_topic_type : pointcloud_topic_type;

  // Reserve space for vectors
  const size_t num_topics = input_topics.size();
  point_cloud_subscribers_.reserve(num_topics);
  point_cloud_publishers_.reserve(num_topics);
  output_raw_messages_.resize(num_topics);
  output_messages_.resize(num_topics);
  tot_original_sizes_.resize(num_topics, 0);
  tot_compressed_sizes_.resize(num_topics, 0);
  message_counts_.resize(num_topics, 0);

  // Create subscribers and publishers for each topic pair
  for (size_t i = 0; i < num_topics; ++i) {
    const auto& input_topic = input_topics[i];
    const auto& output_topic = output_topics[i];

    // Initialize point cloud type support
    auto publisher_info = this->get_publishers_info_by_topic(input_topic);
    auto detected_qos = adapt_request_to_offers(input_topic, publisher_info);

    // Create callback with topic index
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
        std::bind(&CloudiniPointcloudConverterMulti::callback, this, i, std::placeholders::_1);

    RCLCPP_INFO(
        this->get_logger(), "[%zu] Subscribing to topic '%s' of type '%s'", i, input_topic.c_str(),
        input_topic_type.c_str());

    // Create a generic subscriber for point cloud messages
    auto subscriber = this->create_generic_subscription(
        input_topic,       //
        input_topic_type,  //
        detected_qos,      //
        callback);
    point_cloud_subscribers_.push_back(subscriber);

    RCLCPP_INFO(
        this->get_logger(), "[%zu] Publishing to topic '%s' of type '%s'", i, output_topic.c_str(),
        output_topic_type.c_str());

    // Create a generic publisher for point cloud messages
    auto publisher = this->create_generic_publisher(output_topic, output_topic_type, detected_qos);
    point_cloud_publishers_.push_back(publisher);
  }

  RCLCPP_INFO(this->get_logger(), "Successfully initialized %zu topic pairs", num_topics);
}

void CloudiniPointcloudConverterMulti::callback(size_t topic_index, std::shared_ptr<rclcpp::SerializedMessage> msg) {
  // Skip processing if there are no subscribers
  if (point_cloud_publishers_[topic_index]->get_subscription_count() == 0) {
    return;
  }

  // STEP 1: convert the buffer to a ConstBufferView (this is not a copy)
  const auto& input_msg = msg->get_rcl_serialized_message();
  const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

  // STEP 2: extract information from the raw DDS message
  auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

  if (compressing_) {
    const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
    cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolution_);
    cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output_raw_messages_[topic_index]);
  } else {
    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, output_raw_messages_[topic_index]);
  }

  // STEP 3: publish the output message
  output_messages_[topic_index].get_rcl_serialized_message().buffer_length = output_raw_messages_[topic_index].size();
  output_messages_[topic_index].get_rcl_serialized_message().buffer = output_raw_messages_[topic_index].data();
  point_cloud_publishers_[topic_index]->publish(output_messages_[topic_index]);

  tot_original_sizes_[topic_index] += input_msg.buffer_length;
  tot_compressed_sizes_[topic_index] += output_raw_messages_[topic_index].size();

  int& count = message_counts_[topic_index];
  if (count % 20 == 0) {
    double average_ratio = static_cast<double>(tot_compressed_sizes_[topic_index]) / tot_original_sizes_[topic_index];
    tot_compressed_sizes_[topic_index] = 0;
    tot_original_sizes_[topic_index] = 0;
    RCLCPP_INFO(
        this->get_logger(), "[%zu] Converted %d messages, average compression ratio: %.2f", topic_index, count,
        average_ratio);
  }
  count++;
}

int main(int argc, char** argv) {
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<CloudiniPointcloudConverterMulti>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
