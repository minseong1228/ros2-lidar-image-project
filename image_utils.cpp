#include "embedded_project/image_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace image_utils
{

// =====================================================================
// get_field_offsets
// =====================================================================
FieldOffsets get_field_offsets(const sensor_msgs::msg::PointCloud2 & cloud)
{
  FieldOffsets offsets;

  for (const auto & field : cloud.fields) {
    if      (field.name == "x")         offsets.x         = static_cast<int>(field.offset);
    else if (field.name == "y")         offsets.y         = static_cast<int>(field.offset);
    else if (field.name == "z")         offsets.z         = static_cast<int>(field.offset);
    else if (field.name == "intensity") offsets.intensity = static_cast<int>(field.offset);
  }

  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0) {
    throw std::runtime_error(
      "[image_utils] PointCloud2 메시지에 x, y, z 필드가 존재하지 않습니다.");
  }

  return offsets;
}


// =====================================================================
// make_bev_image
// =====================================================================
cv::Mat make_bev_image(
  const sensor_msgs::msg::PointCloud2 & cloud,
  float min_x, float max_x,
  float min_y, float max_y,
  float cell_size,
  std::vector<std::vector<int>> & pixel_to_indices)
{
  int rows = static_cast<int>(std::round((max_x - min_x) / cell_size));
  int cols = static_cast<int>(std::round((max_y - min_y) / cell_size));

  cv::Mat bev_image = cv::Mat::zeros(rows, cols, CV_8UC1);
  pixel_to_indices.assign(rows * cols, std::vector<int>());

  FieldOffsets offsets = get_field_offsets(cloud);

  const size_t point_step  = cloud.point_step;
  const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t * ptr = cloud.data.data() + i * point_step;

    float x = *reinterpret_cast<const float *>(ptr + offsets.x);
    float y = *reinterpret_cast<const float *>(ptr + offsets.y);

    int u = static_cast<int>((max_y - y) / cell_size);
    int v = static_cast<int>((max_x - x) / cell_size);

    if (u >= 0 && u < cols && v >= 0 && v < rows) {
      bev_image.at<uint8_t>(v, u) = 255;
      pixel_to_indices[v * cols + u].push_back(static_cast<int>(i));
    }
  }

  return bev_image;
}

// =====================================================================
// make_range_image
// =====================================================================
cv::Mat make_range_image(
  const sensor_msgs::msg::PointCloud2 & cloud,
  int img_width,
  int img_height,
  float fov_up,
  float fov_down,
  std::vector<std::vector<int>> & pixel_to_indices)
{
  cv::Mat range_image = cv::Mat::zeros(img_height, img_width, CV_8UC1);
  pixel_to_indices.assign(img_height * img_width, std::vector<int>());

  FieldOffsets offsets = get_field_offsets(cloud);

  const size_t point_step  = cloud.point_step;
  const size_t point_count = static_cast<size_t>(cloud.width) * cloud.height;

  constexpr float MAX_RANGE = 200.0f;
  float fov_total = fov_up - fov_down;

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t * ptr = cloud.data.data() + i * point_step;

    float x = *reinterpret_cast<const float *>(ptr + offsets.x);
    float y = *reinterpret_cast<const float *>(ptr + offsets.y);
    float z = *reinterpret_cast<const float *>(ptr + offsets.z);

    float range     = std::sqrt(x*x + y*y + z*z);
    float azimuth   = std::atan2(y, x);
    float elevation = std::atan2(z, std::sqrt(x*x + y*y));

    if (range < 1e-5f) continue;

    int u = static_cast<int>((azimuth + M_PI) / (2.0 * M_PI) * img_width);
    int v = static_cast<int>((fov_up - elevation) / fov_total * img_height);

    if (u >= 0 && u < img_width && v >= 0 && v < img_height) {
      uint8_t pixel_val = static_cast<uint8_t>(
        std::min(range / MAX_RANGE, 1.0f) * 255.0f);

      // Overwrite with closer points if multiple points come into the same pixel
      if (range_image.at<uint8_t>(v, u) == 0 ||
          pixel_val < range_image.at<uint8_t>(v, u))
      {
        range_image.at<uint8_t>(v, u) = pixel_val;
      }
      pixel_to_indices[v * img_width + u].push_back(static_cast<int>(i));
    }
  }

  return range_image;
}

}  // namespace image_utils