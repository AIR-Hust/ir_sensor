# Readme

Package ir_sensor sử dụng cho hệ thống cảm biến tránh vật cản bằng ir sensor

## Installation

```bash
cd catkin_ws/src
git clone https://github.com/AIR-Hust/ir_sensor.git
```

## Cấu hình

Kiểm tra cấu hình trong file `config/IR_array_params.yaml`. Đặt `port: /dev/port3` và cắm vào cổng usb thứ 3 trên pi.

Kết nối nguồn vào cho mạch cảm biến từ nguồn 5V trực tiếp lấy từ chân đế robot lên. Kiểm tra nguồn phải đạt >5V (~5.1-5.3V) nếu không sẽ gây ra sai số cho cảm biến.

Cần một số package yêu cầu, quá trình cài đặt lỗi sẽ xem thiếu gì để cài thêm.

```bash
cd catkin_ws && catkin_make
```

## Chi tiết

- `src/`
  - `ir_arduino_drivers_pcl.py` va `ir_arduino_node.py` là driver đọc dữ liệu từ cảm biến ir.
  - `ir_safety_controller.py` chương trình chứa thuật toán điều khiển robot.

- `launch/`
  - `IR_sensor.launch`: Chạy test IR, load driver
  - `ir_safety_controller.launch`: Chạy điều khiển robot bằng ir

## Một số thay đổi khác

- Thêm đoạn code sau (line 17 - line 23) trong file `dashgo_driver/launch/driver.launch`:

```bash
  <!-- cmd_vel_mux -->
  <node pkg="nodelet" type="nodelet" name="vel_mux_manager" args="manager"/>
  <node pkg="nodelet" type="nodelet" name="cmd_vel_mux"
        args="load yocs_cmd_vel_mux/CmdVelMuxNodelet vel_mux_manager">
    <param name="yaml_cfg_file" value="$(find ir_sensor)/param/mux_plus.yaml" />
    <remap from="cmd_vel_mux/output" to="smoother_cmd_vel"/>
  </node>
```

- Backup file `dashgo_nav/config/odom/costmap_common_params.yaml`, sau đó thay đổi nội dung file này như sau để thêm `ir_cloudpoint` vào costmap (chưa test)

```yaml
robot_radius: 0.20

obstacle_layer:
  enabled: true
  # max_obstacle_height: 0.6
  max_obstacle_height: 1.0
  min_obstacle_height: 0.0
  obstacle_range: 2.0
  raytrace_range: 5.0
  inflation_radius: 0.25
  combination_method: 1
  observation_sources: laser_scan_sensor sonar_scan_sensor ir_scan_sensor
  track_unknown_space: true

  origin_z: 0.0
  z_resolution: 0.1
  z_voxels: 10
  unknown_threshold: 15
  mark_threshold: 0
  publish_voxel_map: true
  footprint_clearing_enabled: true


  laser_scan_sensor:
    data_type: LaserScan
    topic: /scan
    marking: true
    clearing: true
    expected_update_rate: 0
    min_obstacle_height: 0.20
    max_obstacle_height: 0.30

  sonar_scan_sensor:
    data_type: PointCloud2
    topic: /sonar_cloudpoint
    marking: true
    clearing: true
    min_obstacle_height: 0.11
    max_obstacle_height: 0.2
    observation_persistence: 0.0

#FIXME: Add description ir_scan_sensor here
  ir_scan_sensor:
    data_type: PointCloud2
    topic: /ir_cloudpoint
    marking: true
    clearing: true
    min_obstacle_height: 0.8
    max_obstacle_height: 1.2
    observation_persistence: 0.0

inflation_layer:
  enabled:              true
  cost_scaling_factor:  10.0
  inflation_radius:     0.25

static_layer:
  enabled:              true
  map_topic:            "/map"

sonar_layer:
  enabled:            true
  clear_threshold:    0.6
  mark_threshold:     0.8
  topics: ["/sonar0", "/sonar1", "/sonar2", "/sonar3"]
  clear_on_max_reading: true

#FIXME: ir_layer chua dieu chinh thong so
ir_layer:
  enalbe:           true
  clear_threshold:  0.6
  mark_threshold:   0.8
  topics: ["/IR_sensor_arr"]
  clear_on_mark_reading: true
```

## TODO

- Kiểm tra và calib độ chính xác của các cảm biến
- Test giải thuật tránh vật cản
