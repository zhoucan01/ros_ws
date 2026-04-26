// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/layers/intensity_voxel_layer.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "yaml-cpp/yaml.h"

#define VOXEL_BITS 16

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::Observation;

namespace pb_nav2_costmap_2d
{

namespace
{

std::string readNextToken(std::istream & stream)
{
  std::string token;
  while (stream >> token) {
    if (!token.empty() && token.front() == '#') {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    return token;
  }
  throw std::runtime_error("Unexpected end of PGM file while reading token");
}

}  // namespace

void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(name_ + ".combination_method").as_int();

  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);
  loadFreePassMask(node);

  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() {}

void IntensityVoxelLayer::loadFreePassMask(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  free_pass_mask_enabled_ = node->declare_parameter(name_ + ".free_pass_mask.enabled", false);
  free_pass_mask_yaml_filename_ =
    node->declare_parameter(name_ + ".free_pass_mask.yaml_filename", std::string(""));

  free_pass_mask_frame_.clear();
  free_pass_mask_resolution_ = 0.0;
  free_pass_mask_origin_x_ = 0.0;
  free_pass_mask_origin_y_ = 0.0;
  free_pass_mask_occupied_threshold_ = 0.65;
  free_pass_mask_negate_ = false;
  free_pass_mask_width_ = 0;
  free_pass_mask_height_ = 0;
  free_pass_mask_data_.clear();

  if (!free_pass_mask_enabled_) {
    return;
  }

  if (free_pass_mask_yaml_filename_.empty()) {
    throw std::runtime_error("IntensityVoxelLayer free_pass_mask is enabled but yaml_filename is empty");
  }

  const auto mask_yaml = YAML::LoadFile(free_pass_mask_yaml_filename_);
  const auto image_path_node = mask_yaml["image"];
  const auto resolution_node = mask_yaml["resolution"];
  const auto origin_node = mask_yaml["origin"];
  if (!image_path_node || !resolution_node || !origin_node || !origin_node.IsSequence() ||
    origin_node.size() < 2)
  {
    throw std::runtime_error("IntensityVoxelLayer free_pass_mask yaml is missing image/resolution/origin");
  }

  std::string image_path = image_path_node.as<std::string>();
  if (!image_path.empty() && image_path.front() != '/') {
    const auto slash = free_pass_mask_yaml_filename_.find_last_of('/');
    const std::string base_dir =
      slash == std::string::npos ? std::string(".") : free_pass_mask_yaml_filename_.substr(0, slash);
    image_path = base_dir + "/" + image_path;
  }

  free_pass_mask_resolution_ = resolution_node.as<double>();
  free_pass_mask_origin_x_ = origin_node[0].as<double>();
  free_pass_mask_origin_y_ = origin_node[1].as<double>();
  free_pass_mask_negate_ = mask_yaml["negate"] ? mask_yaml["negate"].as<int>() != 0 : false;
  free_pass_mask_occupied_threshold_ =
    mask_yaml["occupied_thresh"] ? mask_yaml["occupied_thresh"].as<double>() : 0.65;
  free_pass_mask_frame_ =
    node->declare_parameter(name_ + ".free_pass_mask.frame_id", layered_costmap_->getGlobalFrameID());
  free_pass_mask_data_ = loadPgmImage(image_path, free_pass_mask_width_, free_pass_mask_height_);
}

std::vector<uint8_t> IntensityVoxelLayer::loadPgmImage(
  const std::string & image_path, unsigned int & width, unsigned int & height)
{
  std::ifstream file(image_path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open free_pass_mask image: " + image_path);
  }

  const auto magic = readNextToken(file);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error("Only PGM P2/P5 images are supported for free_pass_mask");
  }

  width = static_cast<unsigned int>(std::stoul(readNextToken(file)));
  height = static_cast<unsigned int>(std::stoul(readNextToken(file)));
  const auto max_value = std::stoi(readNextToken(file));
  if (max_value <= 0 || max_value > 255) {
    throw std::runtime_error("Unsupported max value in free_pass_mask image");
  }

  std::vector<uint8_t> image(width * height, 0);
  if (magic == "P5") {
    file.get();
    file.read(reinterpret_cast<char *>(image.data()), static_cast<std::streamsize>(image.size()));
    if (file.gcount() != static_cast<std::streamsize>(image.size())) {
      throw std::runtime_error("Failed to read full binary free_pass_mask image");
    }
  } else {
    for (size_t i = 0; i < image.size(); ++i) {
      image[i] = static_cast<uint8_t>(std::stoi(readNextToken(file)));
    }
  }

  return image;
}

bool IntensityVoxelLayer::isInFreePassMask(double px, double py, const rclcpp::Time & stamp) const
{
  if (!free_pass_mask_enabled_ || free_pass_mask_data_.empty()) {
    return false;
  }

  geometry_msgs::msg::PointStamped source_point;
  source_point.header.frame_id = global_frame_;
  source_point.header.stamp = stamp;
  source_point.point.x = px;
  source_point.point.y = py;
  source_point.point.z = 0.0;

  geometry_msgs::msg::PointStamped mask_point;
  try {
    if (free_pass_mask_frame_ == global_frame_) {
      mask_point = source_point;
    } else {
      tf_->transform(source_point, mask_point, free_pass_mask_frame_);
    }
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double mx = (mask_point.point.x - free_pass_mask_origin_x_) / free_pass_mask_resolution_;
  const double my = (mask_point.point.y - free_pass_mask_origin_y_) / free_pass_mask_resolution_;
  const int cell_x = static_cast<int>(std::floor(mx));
  const int cell_y = static_cast<int>(std::floor(my));
  if (cell_x < 0 || cell_y < 0 || cell_x >= static_cast<int>(free_pass_mask_width_) ||
    cell_y >= static_cast<int>(free_pass_mask_height_))
  {
    return false;
  }

  const unsigned int image_y = free_pass_mask_height_ - 1 - static_cast<unsigned int>(cell_y);
  const unsigned int index = image_y * free_pass_mask_width_ + static_cast<unsigned int>(cell_x);
  const double normalized = static_cast<double>(free_pass_mask_data_[index]) / 255.0;
  const double occupancy = free_pass_mask_negate_ ? normalized : 1.0 - normalized;
  return occupancy >= free_pass_mask_occupied_threshold_;
}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  resetMaps();

  if (!enabled_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;
  current_ = current;

  for (const auto & obs : observations) {
    const double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    const double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*obs.cloud_, "intensity");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i) {
      const double px = *it_x;
      const double py = *it_y;
      const double pz = *it_z;

      if (pz < min_obstacle_height_ || pz > max_obstacle_height_) {
        continue;
      }

      if (*it_i < min_obstacle_intensity_ || *it_i > max_obstacle_intensity_) {
        continue;
      }

      const double sq_dist =
        (px - obs.origin_.x) * (px - obs.origin_.x) +
        (py - obs.origin_.y) * (py - obs.origin_.y) +
        (pz - obs.origin_.z) * (pz - obs.origin_.z);
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) {
        continue;
      }

      const rclcpp::Time point_stamp(obs.cloud_->header.stamp);
      if (isInFreePassMask(px, py, point_stamp)) {
        continue;
      }

      unsigned int mx, my, mz;
      if (pz < origin_z_) {
        if (!worldToMap3D(px, py, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(px, py, pz, mx, my, mz)) {
        continue;
      }

      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        const unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(px, py, min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    const unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  int cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  int cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // namespace pb_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::IntensityVoxelLayer, nav2_costmap_2d::Layer)
