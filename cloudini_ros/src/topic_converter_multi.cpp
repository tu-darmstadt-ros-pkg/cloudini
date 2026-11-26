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

#include <chrono>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <optional>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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
  struct TopicPair {
    std::string input_topic;
    std::string output_topic;
    std::optional<rclcpp::QoS> input_qos;  // of the current subscription
    rclcpp::GenericSubscription::SharedPtr subscriber;
    rclcpp::GenericPublisher::SharedPtr publisher;
  };

  // Subscribes with the QoS that the publishers of the input topic offer, once there is one, and subscribes
  // again when what they offer changes: a reader's QoS cannot change, and a RELIABLE reader receives nothing
  // from a BEST_EFFORT publisher. Called at start and by a timer, for every topic pair.
  void adaptToPublishers(size_t topic_index);

  std::vector<TopicPair> topics_;
  std::string input_topic_type_;
  std::string output_topic_type_;
  rclcpp::TimerBase::SharedPtr graph_poll_timer_;

  // Per-topic buffers
  std::vector<std::vector<uint8_t>> output_raw_messages_;
  std::vector<rclcpp::SerializedMessage> output_messages_;

  bool compressing_ = true;
  std::vector<double> resolutions_;  // per-topic resolution values

  // Per-topic statistics
  std::vector<uint64_t> tot_original_sizes_;
  std::vector<uint64_t> tot_compressed_sizes_;
  std::vector<int> message_counts_;
};
//-----------------------------------------------------

namespace {
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
}  // namespace
//-----------------------------------------------------

CloudiniPointcloudConverterMulti::CloudiniPointcloudConverterMulti(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cloudini_pointcloud_converter_multi", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
  // Declare parameters for input and output topics
  this->declare_parameter<bool>("compressing", true);
  this->declare_parameter<std::vector<std::string>>("topics_input", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("topics_output", std::vector<std::string>{});
  this->declare_parameter<std::vector<double>>("resolution", std::vector<double>{});

  // read parameters
  compressing_ = this->get_parameter("compressing").as_bool();

  const std::vector<std::string> input_topics = this->get_parameter("topics_input").as_string_array();
  if (input_topics.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input topics list is empty or not set");
    throw std::runtime_error("Input topics list is required");
  }

  std::vector<std::string> output_topics = this->get_parameter("topics_output").as_string_array();
  std::vector<double> resolution_params = this->get_parameter("resolution").as_double_array();

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

  input_topic_type_ = compressing_ ? pointcloud_topic_type : compressed_topic_type;
  output_topic_type_ = compressing_ ? compressed_topic_type : pointcloud_topic_type;

  // Handle resolution parameter
  const size_t num_topics = input_topics.size();
  if (resolution_params.empty()) {
    // Case 1: Not specified, use default for all topics
    resolutions_.resize(num_topics, 0.001);
    RCLCPP_INFO(this->get_logger(), "Using default resolution 0.001 for all %zu topics", num_topics);
  } else if (resolution_params.size() == 1) {
    // Case 2: One value specified, use for all topics
    resolutions_.resize(num_topics, resolution_params[0]);
    RCLCPP_INFO(this->get_logger(), "Using resolution %.6f for all %zu topics", resolution_params[0], num_topics);
  } else if (resolution_params.size() == num_topics) {
    // Case 3: Multiple values, must match number of topics
    resolutions_ = resolution_params;
    RCLCPP_INFO(this->get_logger(), "Using per-topic resolution values for %zu topics", num_topics);
  } else {
    RCLCPP_ERROR(
        this->get_logger(), "Resolution parameter size (%zu) must be 0, 1, or match topics size (%zu)",
        resolution_params.size(), num_topics);
    throw std::runtime_error("Resolution parameter size mismatch");
  }

  topics_.resize(num_topics);
  output_raw_messages_.resize(num_topics);
  output_messages_.resize(num_topics);
  tot_original_sizes_.resize(num_topics, 0);
  tot_compressed_sizes_.resize(num_topics, 0);
  message_counts_.resize(num_topics, 0);

  for (size_t i = 0; i < num_topics; ++i) {
    topics_[i].input_topic = input_topics[i];
    topics_[i].output_topic = output_topics[i];
    adaptToPublishers(i);
    if (!topics_[i].input_qos) {
      RCLCPP_INFO(this->get_logger(), "[%zu] Waiting for a publisher on topic '%s'", i, input_topics[i].c_str());
    }
  }
  graph_poll_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this] {
    for (size_t i = 0; i < topics_.size(); ++i) {
      adaptToPublishers(i);
    }
  });

  RCLCPP_INFO(this->get_logger(), "Successfully initialized %zu topic pairs", num_topics);
}

void CloudiniPointcloudConverterMulti::adaptToPublishers(size_t topic_index) {
  TopicPair& topic = topics_[topic_index];
  const auto publishers = this->get_publishers_info_by_topic(topic.input_topic);
  if (publishers.empty()) {
    return;  // nothing to adapt to: keep the current subscription, if any
  }
  rclcpp::QoS qos = adapt_request_to_offers(topic.input_topic, publishers);
  if (topic.input_qos && topic.input_qos->durability() == rclcpp::DurabilityPolicy::Volatile) {
    // A VOLATILE reader matches every publisher: subscribing again as TRANSIENT_LOCAL would only receive
    // the clouds already converted a second time.
    qos.durability_volatile();
  }
  if (topic.input_qos == qos) {
    return;
  }
  const bool reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
  const bool transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;
  RCLCPP_INFO(
      this->get_logger(), "[%zu] Subscribing to topic '%s' of type '%s' (%s, %s, resolution: %.6f)", topic_index,
      topic.input_topic.c_str(), input_topic_type_.c_str(), reliable ? "RELIABLE" : "BEST_EFFORT",
      transient_local ? "TRANSIENT_LOCAL" : "VOLATILE", resolutions_[topic_index]);
  // The publisher exists before the first message arrives. Its QoS stays, so that the subscribers of the
  // output keep matching.
  if (!topic.publisher) {
    RCLCPP_INFO(
        this->get_logger(), "[%zu] Publishing to topic '%s' of type '%s'", topic_index, topic.output_topic.c_str(),
        output_topic_type_.c_str());
    topic.publisher = this->create_generic_publisher(topic.output_topic, output_topic_type_, qos);
  }
  std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback =
      std::bind(&CloudiniPointcloudConverterMulti::callback, this, topic_index, std::placeholders::_1);
  topic.subscriber = this->create_generic_subscription(topic.input_topic, input_topic_type_, qos, callback);
  topic.input_qos = qos;
}

void CloudiniPointcloudConverterMulti::callback(size_t topic_index, std::shared_ptr<rclcpp::SerializedMessage> msg) {
  // Skip processing if there are no subscribers
  if (topics_[topic_index].publisher->get_subscription_count() == 0) {
    return;
  }

  // STEP 1: convert the buffer to a ConstBufferView (this is not a copy)
  const auto& input_msg = msg->get_rcl_serialized_message();
  const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

  // STEP 2: extract information from the raw DDS message
  auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

  if (compressing_) {
    cloudini_ros::applyResolutionProfile(cloudini_ros::ResolutionProfile{}, pc_info.fields, resolutions_[topic_index]);
    const auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);
    cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output_raw_messages_[topic_index]);
  } else {
    cloudini_ros::convertCompressedCloudToPointCloud2(pc_info, output_raw_messages_[topic_index]);
  }

  // STEP 3: publish the output message
  output_messages_[topic_index].get_rcl_serialized_message().buffer_length = output_raw_messages_[topic_index].size();
  output_messages_[topic_index].get_rcl_serialized_message().buffer = output_raw_messages_[topic_index].data();
  topics_[topic_index].publisher->publish(output_messages_[topic_index]);

  tot_original_sizes_[topic_index] += input_msg.buffer_length;
  tot_compressed_sizes_[topic_index] += output_raw_messages_[topic_index].size();

  int& count = message_counts_[topic_index];
  if (count % 20 == 0) {
    double average_ratio = static_cast<double>(tot_compressed_sizes_[topic_index]) / tot_original_sizes_[topic_index];
    tot_compressed_sizes_[topic_index] = 0;
    tot_original_sizes_[topic_index] = 0;
    RCLCPP_DEBUG(
        this->get_logger(), "[%zu] Converted %d messages, average compression ratio: %.2f", topic_index, count,
        average_ratio);
  }
  count++;
}

RCLCPP_COMPONENTS_REGISTER_NODE(CloudiniPointcloudConverterMulti)
