/**
 * @file detection_scene_builder.cpp
 * @brief Node that subscribes to Detection3DArray and updates MoveIt planning scene
 *
 * This node receives 3D object detections and dynamically adds/updates/removes
 * collision objects in the MoveIt2 planning scene.
 */

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>

class DetectionSceneBuilder : public rclcpp::Node
{
public:
  DetectionSceneBuilder()
  : Node("detection_scene_builder")
  {
    // Declare parameters
    this->declare_parameter("detections_topic", "/detections_3d");
    this->declare_parameter("planning_frame", "world");
    this->declare_parameter("stale_timeout", 2.0);
    this->declare_parameter("update_rate", 2.0);
    this->declare_parameter("padding", 0.01);
    this->declare_parameter("min_object_size", 0.02);

    // Get parameters
    std::string detections_topic = this->get_parameter("detections_topic").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    stale_timeout_ = this->get_parameter("stale_timeout").as_double();
    double update_rate = this->get_parameter("update_rate").as_double();
    padding_ = this->get_parameter("padding").as_double();
    min_object_size_ = this->get_parameter("min_object_size").as_double();

    // Create planning scene interface
    psi_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

    // Subscribe to detections
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
      detections_topic,
      10,
      std::bind(&DetectionSceneBuilder::detections_callback, this, std::placeholders::_1));

    // Timer for rate-limited planning scene updates
    auto period = std::chrono::duration<double>(1.0 / update_rate);
    update_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DetectionSceneBuilder::update_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Detection Scene Builder initialized");
    RCLCPP_INFO(this->get_logger(), "  Detections topic: %s", detections_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Planning frame: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz", update_rate);
    RCLCPP_INFO(this->get_logger(), "  Stale timeout: %.1f s", stale_timeout_);
  }

private:
  void detections_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Store latest detections for processing in timer callback
    pending_detections_ = msg->detections;
    last_detection_time_ = this->now();
  }

  void update_timer_callback()
  {
    std::vector<vision_msgs::msg::Detection3D> detections_to_process;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      detections_to_process = std::move(pending_detections_);
      pending_detections_.clear();
    }

    // Process detections and update planning scene
    if (!detections_to_process.empty())
    {
      process_detections(detections_to_process);
    }

    // Remove stale objects
    remove_stale_objects();
  }

  void process_detections(const std::vector<vision_msgs::msg::Detection3D>& detections)
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    auto now = this->now();

    for (const auto& detection : detections)
    {
      // Generate or use existing ID
      std::string object_id = detection.id;
      if (object_id.empty())
      {
        object_id = generate_object_id(detection);
      }

      // Create collision object
      auto collision_object = create_collision_object(detection, object_id);
      collision_objects.push_back(collision_object);

      // Update tracking
      tracked_objects_[object_id] = now;
    }

    // Apply all collision objects at once
    if (!collision_objects.empty())
    {
      psi_->applyCollisionObjects(collision_objects);
      RCLCPP_DEBUG(this->get_logger(), "Updated %zu collision objects", collision_objects.size());
    }
  }

  moveit_msgs::msg::CollisionObject create_collision_object(
    const vision_msgs::msg::Detection3D& detection,
    const std::string& object_id)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_id;
    object.header.frame_id = planning_frame_;
    object.header.stamp = this->now();
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Create BOX primitive
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);

    // Apply padding and minimum size constraints
    primitive.dimensions[0] = std::max(detection.bbox.size.x + padding_, min_object_size_);
    primitive.dimensions[1] = std::max(detection.bbox.size.y + padding_, min_object_size_);
    primitive.dimensions[2] = std::max(detection.bbox.size.z + padding_, min_object_size_);

    object.primitives.push_back(primitive);

    // Use pose from bounding box center
    geometry_msgs::msg::Pose pose;
    pose.position = detection.bbox.center.position;
    pose.orientation = detection.bbox.center.orientation;

    // Ensure valid quaternion
    if (pose.orientation.w == 0.0 && pose.orientation.x == 0.0 &&
        pose.orientation.y == 0.0 && pose.orientation.z == 0.0)
    {
      pose.orientation.w = 1.0;
    }

    object.primitive_poses.push_back(pose);

    return object;
  }

  std::string generate_object_id(const vision_msgs::msg::Detection3D& detection)
  {
    // Use class name from first hypothesis if available
    std::string class_name = "unknown";
    if (!detection.results.empty())
    {
      class_name = detection.results[0].hypothesis.class_id;
    }

    // Generate unique ID based on position hash
    int hash = static_cast<int>(
      std::abs(detection.bbox.center.position.x * 1000) +
      std::abs(detection.bbox.center.position.y * 1000) +
      std::abs(detection.bbox.center.position.z * 1000)) % 1000;

    return "det_" + class_name + "_" + std::to_string(hash);
  }

  void remove_stale_objects()
  {
    auto now = this->now();
    std::vector<std::string> to_remove;

    for (const auto& [id, last_seen] : tracked_objects_)
    {
      double elapsed = (now - last_seen).seconds();
      if (elapsed > stale_timeout_)
      {
        to_remove.push_back(id);
      }
    }

    if (!to_remove.empty())
    {
      psi_->removeCollisionObjects(to_remove);

      for (const auto& id : to_remove)
      {
        tracked_objects_.erase(id);
        RCLCPP_INFO(this->get_logger(), "Removed stale object: %s", id.c_str());
      }
    }
  }

  // Subscriber
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;

  // Timer for rate-limited updates
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Planning scene interface
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_;

  // Object tracking: object_id -> last_seen_time
  std::map<std::string, rclcpp::Time> tracked_objects_;

  // Pending detections buffer
  std::vector<vision_msgs::msg::Detection3D> pending_detections_;
  rclcpp::Time last_detection_time_;
  std::mutex mutex_;

  // Parameters
  std::string planning_frame_;
  double stale_timeout_;
  double padding_;
  double min_object_size_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectionSceneBuilder>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
