#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdexcept>
#include <string>

namespace image_utils
{

// PointCloud2 메시지에서 x, y, z, intensity 필드의 byte offset을 저장하는 구조체
struct FieldOffsets
{
  int x         = -1;
  int y         = -1;
  int z         = -1;
  int intensity = -1;
};

/**
 * @brief PointCloud2 메시지의 fields에서 각 채널의 byte offset을 파싱한다.
 *
 * @param cloud  입력 PointCloud2 메시지
 * @return FieldOffsets  각 필드의 offset (없는 필드는 -1)
 * @throws std::runtime_error  x, y, z 중 하나라도 없으면 예외 발생
 */
FieldOffsets get_field_offsets(const sensor_msgs::msg::PointCloud2 & cloud);


/**
 * @brief PointCloud2를 XY 평면에 투영하여 BEV(Bird's Eye View) 이미지를 생성한다.
 *
 * - 이미지 좌표계: 왼쪽 위가 원점, x축 오른쪽(+y 방향), y축 아래(-x 방향)
 * - 픽셀 값: 해당 셀에 포인트가 존재하면 255 (흰색), 없으면 0 (검정)
 *
 * @param cloud             입력 PointCloud2 메시지
 * @param min_x             ROI x 최솟값 (미터)
 * @param max_x             ROI x 최댓값 (미터)
 * @param min_y             ROI y 최솟값 (미터)
 * @param max_y             ROI y 최댓값 (미터)
 * @param cell_size         셀 하나의 크기 (미터, 정사각형)
 * @param pixel_to_indices  [출력] 픽셀 인덱스(v * cols + u) → 포인트 인덱스 목록
 * @return cv::Mat          생성된 BEV 이미지 (CV_8UC1)
 *
 * [TODO] 아래 두 단계를 직접 구현하시오.
 *   1. 각 포인트의 (x, y) 좌표를 픽셀 좌표 (u, v)로 변환
 *   2. ROI 범위 안에 있는 포인트만 이미지에 표시하고 pixel_to_indices에 인덱스 기록
 */
cv::Mat make_bev_image(
  const sensor_msgs::msg::PointCloud2 & cloud,
  float min_x, float max_x,
  float min_y, float max_y,
  float cell_size,
  std::vector<std::vector<int>> & pixel_to_indices);


/**
 * @brief PointCloud2를 구형 투영(Spherical Projection)하여 Range Image를 생성한다.
 *
 * - 수평각 (azimuth)  : atan2(y, x)              → 이미지 u축 (가로)
 * - 수직각 (elevation): atan2(z, sqrt(x²+y²))    → 이미지 v축 (세로)
 * - 픽셀 값: 해당 방향의 거리(range)를 0~255로 정규화한 값
 *
 * @param cloud             입력 PointCloud2 메시지
 * @param img_width         출력 이미지 가로 픽셀 수
 * @param img_height        출력 이미지 세로 픽셀 수 (= 라이다 채널 수)
 * @param fov_up            수직 FOV 상단 (라디안)
 * @param fov_down          수직 FOV 하단 (라디안, 음수)
 * @param pixel_to_indices  [출력] 픽셀 인덱스(v * img_width + u) → 포인트 인덱스 목록
 * @return cv::Mat          생성된 Range Image (CV_8UC1)
 *
 * [TODO] 아래 두 단계를 직접 구현하시오.
 *   1. 각 포인트의 (x, y, z)로부터 azimuth, elevation, range를 계산
 *   2. azimuth, elevation을 픽셀 좌표 (u, v)로 변환 후 range 값을 픽셀에 기록하고
 *      pixel_to_indices에 인덱스 기록
 */
cv::Mat make_range_image(
  const sensor_msgs::msg::PointCloud2 & cloud,
  int img_width,
  int img_height,
  float fov_up,
  float fov_down,
  std::vector<std::vector<int>> & pixel_to_indices);

}  // namespace image_utils