# ROS 2 LiDAR Image Projection & ROI Filtering

ROS 2 환경에서 LiDAR `PointCloud2` 데이터를 이미지 형태로 투영하고, ROI(Region of Interest)를 지정하여 해당 영역의 PointCloud를 필터링하는 프로젝트입니다.

## 주요 기능

- LiDAR `PointCloud2` 데이터 구독
- BEV(Bird's Eye View) 이미지 생성
- Range Image 생성
- ROI 지정 및 관리
- ROI 영역에 해당하는 PointCloud 필터링
- RViz를 이용한 투영 이미지 및 필터링 결과 확인

## 프로젝트 구조

```text
.
├── README.md
├── .gitignore
├── embedded_project2/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── image_parameter.yaml
│   │   ├── student_bev.yaml
│   │   └── student_range.yaml
│   ├── include/
│   │   └── embedded_project/
│   │       └── image_utils.hpp
│   ├── src/
│   │   ├── image_publisher.cpp
│   │   ├── image_utils.cpp
│   │   └── roi_selector.cpp
│   └── srv/
│       └── SetROI.srv
└── embedded_project.rviz
```

## 주요 구성

### `image_publisher`

`/point_cloud` 토픽으로 들어오는 `sensor_msgs/PointCloud2` 데이터를 받아 이미지로 변환합니다.

- BEV 모드: XY 평면 기준으로 PointCloud를 투영
- RANGE 모드: LiDAR의 수평각과 수직각을 이용하여 Range Image 생성
- 생성된 이미지는 `/projected_image`로 publish
- ROI가 설정되면 해당 영역의 PointCloud를 `/filtered_cloud`로 publish

### `roi_selector`

터미널에서 ROI 좌표를 입력받아 `SetROI` 서비스를 호출합니다.

입력 형식:

```text
x1 y1 x2 y2
```

### `SetROI`

ROI 좌표를 전달하기 위한 ROS 2 Service Interface입니다.

```text
int32 x1
int32 y1
int32 x2
int32 y2
---
bool success
string message
```

## 주요 토픽

| Topic | Type | 설명 |
|---|---|---|
| `/point_cloud` | `sensor_msgs/PointCloud2` | 입력 LiDAR PointCloud |
| `/projected_image` | `sensor_msgs/Image` | BEV 또는 Range Image |
| `/filtered_cloud` | `sensor_msgs/PointCloud2` | ROI에 의해 필터링된 PointCloud |

## 주요 파라미터

### BEV

기본 설정에서는 다음 범위를 사용합니다.

```yaml
min_x: -40.0
max_x: 40.0
min_y: -40.0
max_y: 40.0
cell_size: 0.1
```

### Range Image

```yaml
width: 1024
height: 64
```

세부 설정은 `config/` 아래의 YAML 파일에서 확인할 수 있습니다.

## 빌드

ROS 2 작업공간의 `src` 디렉터리에 프로젝트를 넣은 뒤 다음과 같이 빌드합니다.

```bash
cd ~/project_ws
colcon build --packages-select embedded_project
source install/setup.bash
```

## 실행

### Image Publisher

```bash
ros2 run embedded_project image_publisher
```

YAML 설정을 사용하는 경우:

```bash
ros2 run embedded_project image_publisher --ros-args --params-file src/embedded_project2/config/student_bev.yaml
```

또는 Range Image 설정:

```bash
ros2 run embedded_project image_publisher --ros-args --params-file src/embedded_project2/config/student_range.yaml
```

### ROI Selector

`image_publisher`가 실행된 상태에서:

```bash
ros2 run embedded_project roi_selector
```

터미널에 ROI 좌표를 입력합니다.

```text
Enter ROI (x1 y1 x2 y2):
```

## RViz 확인

RViz에서 다음 토픽을 확인할 수 있습니다.

- `/projected_image`
- `/filtered_cloud`

프로젝트에 포함된 RViz 설정 파일을 이용하면 관련 Display 설정을 불러올 수 있습니다.

## 개발 환경

- ROS 2
- C++
- OpenCV
- cv_bridge
- sensor_msgs
- RViz

