#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "embedded_project/image_utils.hpp"
#include "embedded_project/srv/set_roi.hpp"

class ImagePublisherNode : public rclcpp::Node
{
public:
  ImagePublisherNode() : Node("image_publisher")
  {

this->declare_parameter<std::string>("mode", "BEV");

this->declare_parameter<float>("bev.min_x", -40.0f);
this->declare_parameter<float>("bev.max_x", 40.0f);
this->declare_parameter<float>("bev.min_y", -40.0f);
this->declare_parameter<float>("bev.max_y", 40.0f);
this->declare_parameter<float>("bev.cell_size", 0.1f);

this->declare_parameter<int>("range_image.width", 1024);
this->declare_parameter<int>("range_image.height", 64);

this->declare_parameter<float>("range_image.fov_up",
                               2.0f * M_PI / 180.0f);
this->declare_parameter<float>("range_image.fov_down",
                               -24.8f * M_PI / 180.0f);

mode_ = this->get_parameter("mode").as_string();

min_x_ = static_cast<float>(this->get_parameter("bev.min_x").as_double());
max_x_ = static_cast<float>(this->get_parameter("bev.max_x").as_double());
min_y_ = static_cast<float>(this->get_parameter("bev.min_y").as_double());
max_y_ = static_cast<float>(this->get_parameter("bev.max_y").as_double());
cell_size_ = static_cast<float>(this->get_parameter("bev.cell_size").as_double());

range_width_ = this->get_parameter("range_image.width").as_int();
range_height_ = this->get_parameter("range_image.height").as_int();

fov_up_ = static_cast<float>(this->get_parameter("range_image.fov_up").as_double());
fov_down_ = static_cast<float>(this->get_parameter("range_image.fov_down").as_double());
  
    if (mode_ != "BEV" && mode_ != "RANGE") {
      throw std::runtime_error(
        "[image_publisher] mode는 'BEV' 또는 'RANGE'여야 합니다. 현재 값: " + mode_);
    }

    if (mode_ == "BEV") {
      int x_cells = static_cast<int>(std::round((max_x_ - min_x_) / cell_size_));
      int y_cells = static_cast<int>(std::round((max_y_ - min_y_) / cell_size_));

      if (x_cells <= 0 || y_cells <= 0) {
        throw std::runtime_error(
          "[image_publisher] ROI 범위 또는 cell_size가 올바르지 않습니다. "
          "image_parameters.yaml을 확인하세요.");
      }
    }

    // ---------------------------------------------------------------
    // Publisher / Subscriber / Service Generation
    // ---------------------------------------------------------------
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/projected_image", 10);

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_cloud", 10);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/point_cloud", 10,
      std::bind(&ImagePublisherNode::pointCloudCallback, this, std::placeholders::_1));

    roi_service_ = this->create_service<embedded_project::srv::SetROI>(
      "SetROI",
      std::bind(&ImagePublisherNode::roiServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ImagePublisherNode 시작 - mode: %s", mode_.c_str());
  }

private:
  void roiServiceCallback(
    const std::shared_ptr<embedded_project::srv::SetROI::Request> request,
    std::shared_ptr<embedded_project::srv::SetROI::Response> response)
  {
    if (request->x1 < 0 || request->y1 < 0 ||
        request->x2 >= image_width_ || request->y2 >= image_height_ ||
        request->x1 >= request->x2 || request->y1 >= request->y2)
    {
      response->success = false;
      response->message = "유효하지 않은 ROI 범위입니다. 이미지 크기: " +
        std::to_string(image_width_) + " x " + std::to_string(image_height_);
      return;
    }

    std::array<int32_t, 4> roi = {request->x1, request->y1, request->x2, request->y2};
    roi_list_.push_back(roi);

    RCLCPP_INFO(this->get_logger(), "ROI 추가: (%d, %d) ~ (%d, %d)  총 %zu개",
      request->x1, request->y1, request->x2, request->y2, roi_list_.size());

    response->success = true;
    response->message = "ROI 추가 완료. 총 " + std::to_string(roi_list_.size()) + "개 적용 중";
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_cloud_ = msg;

    try {
      cv::Mat image;
      std::string encoding;

      if (mode_ == "BEV") {
        image_width_  = static_cast<int>(std::round((max_y_ - min_y_) / cell_size_));
        image_height_ = static_cast<int>(std::round((max_x_ - min_x_) / cell_size_));
        image    = image_utils::make_bev_image(*msg, min_x_, max_x_, min_y_, max_y_, cell_size_, pixel_to_indices_);
        encoding = "mono8";
      } else {
        image_width_  = range_width_;
        image_height_ = range_height_;
        cv::Mat raw   = image_utils::make_range_image(*msg, range_width_, range_height_, fov_up_, fov_down_, pixel_to_indices_);
        cv::applyColorMap(raw, image, cv::COLORMAP_TURBO);
        encoding = "bgr8";
      }

      if (!roi_list_.empty()) {
        cv::Mat display;
        if (mode_ == "BEV") {
          cv::cvtColor(image, display, cv::COLOR_GRAY2BGR);
        } else {
          display = image.clone();
        }

        for (const auto & roi : roi_list_) {
          cv::rectangle(display,
            cv::Point(roi[0], roi[1]),
            cv::Point(roi[2], roi[3]),
            cv::Scalar(0, 0, 255),  // BGR Red
            2);
        }

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", display).toImageMsg());
      } else {
        image_pub_->publish(*cv_bridge::CvImage(msg->header, encoding, image).toImageMsg());
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }

    // 'filtered_cloud publish' if at least one ROI is registered
    if (!roi_list_.empty() && last_cloud_) {
      publishFilteredCloud();
    }
  }

  void publishFilteredCloud()
  {
    const size_t point_step = last_cloud_->point_step;

    std::vector<uint8_t> out_data;
    out_data.reserve(last_cloud_->data.size());

    // Visited flag to prevent duplicate points
    std::vector<bool> visited(pixel_to_indices_.size(), false);

    for (const auto & roi : roi_list_) {
      for (int v = roi[1]; v <= roi[3]; ++v) {
        for (int u = roi[0]; u <= roi[2]; ++u) {
          int pixel_idx = v * image_width_ + u;
          if (pixel_idx < 0 || pixel_idx >= static_cast<int>(pixel_to_indices_.size())) continue;
          if (visited[pixel_idx]) continue;
          visited[pixel_idx] = true;

          for (int idx : pixel_to_indices_[pixel_idx]) {
            const uint8_t * ptr = last_cloud_->data.data() + idx * point_step;
            out_data.insert(out_data.end(), ptr, ptr + point_step);
          }
        }
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header       = last_cloud_->header;
    output.height       = 1;
    output.width        = static_cast<uint32_t>(out_data.size() / point_step);
    output.fields       = last_cloud_->fields;
    output.is_bigendian = last_cloud_->is_bigendian;
    output.point_step   = point_step;
    output.row_step     = output.width * point_step;
    output.is_dense     = false;
    output.data         = std::move(out_data);

    filtered_cloud_pub_->publish(output);
  }

  // ---------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------
  std::string mode_;

  // BEV Parameters
  float min_x_, max_x_, min_y_, max_y_, cell_size_;

  // Range Image Parameters
  int   range_width_, range_height_;
  float fov_up_, fov_down_;

  // Current Image Size
  int image_width_  = 0;
  int image_height_ = 0;

  // Pixel → Point index mapping (update every frame)
  std::vector<std::vector<int>> pixel_to_indices_;

  // Point cloud last received
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_;

  // Cumulative ROI list (x1, y1, x2, y2)
  std::vector<std::array<int32_t, 4>> roi_list_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  rclcpp::Service<embedded_project::srv::SetROI>::SharedPtr roi_service_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
