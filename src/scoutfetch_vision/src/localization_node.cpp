// SPDX-License-Identifier: Apache-2.0
#include <scoutfetch_vision/localization_node.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace scoutfetch_vision
{

namespace
{
constexpr double kMinQuaternionNorm = 1e-6;
constexpr char kWindowName[] = "D435i click target";
}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this)
{
  params_.pixel_u = this->declare_parameter<int>("pixel_u", params_.pixel_u);
  params_.pixel_v = this->declare_parameter<int>("pixel_v", params_.pixel_v);
  params_.pixel_source =
    this->declare_parameter<std::string>("pixel_source", params_.pixel_source);
  if (params_.pixel_source != "yaml" && params_.pixel_source != "click" &&
      params_.pixel_source != "yolo") {
    RCLCPP_WARN(this->get_logger(), "pixel_source=%s 无效, 回退到 yaml", params_.pixel_source.c_str());
    params_.pixel_source = "yaml";
  }
  params_.detection_topic =
    this->declare_parameter<std::string>("detection_topic", params_.detection_topic);
  params_.detection_timeout =
    this->declare_parameter<double>("detection_timeout", params_.detection_timeout);
  params_.max_detection_depth_delta = this->declare_parameter<double>(
    "max_detection_depth_delta", params_.max_detection_depth_delta);
  params_.grasp_offset_ratio =
    this->declare_parameter<double>("grasp_offset_ratio", params_.grasp_offset_ratio);
  params_.color_topic =
    this->declare_parameter<std::string>("color_topic", params_.color_topic);
  params_.depth_topic =
    this->declare_parameter<std::string>("depth_topic", params_.depth_topic);
  params_.depth_info_topic =
    this->declare_parameter<std::string>("depth_info_topic", params_.depth_info_topic);
  params_.optical_frame =
    this->declare_parameter<std::string>("optical_frame", params_.optical_frame);
  params_.target_frame =
    this->declare_parameter<std::string>("target_frame", params_.target_frame);
  params_.min_depth = this->declare_parameter<double>("min_depth", params_.min_depth);
  params_.max_depth = this->declare_parameter<double>("max_depth", params_.max_depth);
  params_.orient_w = this->declare_parameter<double>("orient_w", params_.orient_w);
  params_.orient_x = this->declare_parameter<double>("orient_x", params_.orient_x);
  params_.orient_y = this->declare_parameter<double>("orient_y", params_.orient_y);
  params_.orient_z = this->declare_parameter<double>("orient_z", params_.orient_z);
  const double rate = this->declare_parameter<double>("publish_rate", params_.publish_rate);
  params_.marker_scale = this->declare_parameter<double>("marker_scale", params_.marker_scale);
  params_.marker_frame =
    this->declare_parameter<std::string>("marker_frame", params_.marker_frame);

  publish_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(rate, 1.0)));
  last_publish_ = std::chrono::steady_clock::now() - publish_interval_;

  // D435i 发布 RELIABLE 图像, 订阅须 RELIABLE (不能直接用 SensorDataQoS 的 BEST_EFFORT)
  const auto img_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::Volatile);

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    params_.depth_info_topic, img_qos,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    params_.depth_topic, img_qos,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { depthCallback(msg); });

  target_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
  // /target_pose_map: reliable + transient_local + keep_last(1),
  // 使后启动的 scoutfetch_mission 能拿到最后一次点击的世界系目标
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::TransientLocal);
  target_pose_map_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_map", map_qos);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

  RCLCPP_INFO(this->get_logger(),
              "localization_node: pixel_source=%s depth=%s info=%s pixel=(%d,%d) optical=%s "
              "target=%s",
              params_.pixel_source.c_str(), params_.depth_topic.c_str(),
              params_.depth_info_topic.c_str(), params_.pixel_u, params_.pixel_v,
              params_.optical_frame.c_str(), params_.target_frame.c_str());

  // ---- click 模式: OpenCV 窗口 + 鼠标左键选点 ----
  if (params_.pixel_source == "click") {
    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      params_.color_topic, img_qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { colorCallback(msg); });
    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kWindowName, &LocalizationNode::mouseCallbackCb, this);
    display_thread_ = std::thread([this]() { displayLoop(); });
    RCLCPP_INFO(this->get_logger(),
                "click 模式: 窗口 '%s' 已打开, 鼠标左键点击目标中心选点", kWindowName);
  }

  // ---- yolo 模式: 目标像素来自视觉检测节点 (/detections), 与机器人位姿无关 ----
  if (params_.pixel_source == "yolo") {
    detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      params_.detection_topic, rclcpp::QoS(10),
      [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        detectionsCallback(msg);
      });
    RCLCPP_INFO(this->get_logger(),
                "yolo 模式: detections=%s 检测有效期=%.2fs; 无有效检测时不发布目标",
                params_.detection_topic.c_str(), params_.detection_timeout);
  }
}

LocalizationNode::~LocalizationNode()
{
  stop_display_ = true;
  if (display_thread_.joinable()) {
    display_thread_.join();
  }
}

void LocalizationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(info_mutex_);
  latest_camera_info_ = *msg;
}

void LocalizationNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (params_.pixel_source == "yolo") {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    depth_history_.push_back(msg);
    while (depth_history_.size() > 30) {
      depth_history_.pop_front();
    }
  }
  if (params_.pixel_source == "click") {
    // click 模式: 只保存最新深度帧, 由鼠标点击触发处理
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_depth_ = msg;
    return;
  }

  // yaml / yolo 模式: 节流后每周期处理一次
  const auto now = std::chrono::steady_clock::now();
  if (now - last_publish_ < publish_interval_) {
    return;
  }
  last_publish_ = now;

  // yolo 模式: 目标像素来自检测框中心, 深度仍取对齐后的深度图
  if (params_.pixel_source == "yolo") {
    processDetectedTarget();
    return;
  }
  processPixelAt(params_.pixel_u, params_.pixel_v, *msg);
}

void LocalizationNode::detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // 只保留置信度最高的一条作为本轮目标; 检测节点已按类别过滤, 这里不再判类别
  const vision_msgs::msg::Detection2D * best = nullptr;
  double best_score = -1.0;
  for (const auto & detection : msg->detections) {
    const double score =
      detection.results.empty() ? 0.0 : detection.results.front().hypothesis.score;
    if (score > best_score) {
      best_score = score;
      best = &detection;
    }
  }
  if (best == nullptr) {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    latest_detection_.reset();
    return;
  }
  std::lock_guard<std::mutex> lock(detection_mutex_);
  latest_detection_ = *best;
  latest_detection_stamp_ = msg->header.stamp;
}

bool LocalizationNode::processDetectedTarget()
{
  vision_msgs::msg::Detection2D detection;
  rclcpp::Time detection_stamp;
  {
    std::lock_guard<std::mutex> lock(detection_mutex_);
    if (!latest_detection_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "yolo 模式: 尚未收到检测 (%s)", params_.detection_topic.c_str());
      return false;
    }
    detection = *latest_detection_;
    detection_stamp = latest_detection_stamp_;
  }

  if (processed_detection_stamp_ && *processed_detection_stamp_ == detection_stamp.nanoseconds()) {
    return false;
  }
  // Inference arrives late. Match its original RGB timestamp to buffered depth,
  // instead of projecting an old pixel into the newest (rotated) viewpoint.
  sensor_msgs::msg::Image::SharedPtr depth_img;
  double frame_delta = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    for (const auto & frame : depth_history_) {
      const double delta = std::abs((rclcpp::Time(frame->header.stamp) - detection_stamp).seconds());
      if (delta < frame_delta) {
        frame_delta = delta;
        depth_img = frame;
      }
    }
  }
  if (!depth_img) {
    return false;
  }

  // 检测过期意味着目标已离开视野 (或检测节点停了)。继续用旧像素会发一个错误目标,
  // 而 task_fsm 会把收到的第一条 /target_pose_map 冻结成本次任务目标。
  const double age = (this->now() - detection_stamp).seconds();
  if (age < -0.05 || age > params_.detection_timeout ||
      frame_delta > params_.max_detection_depth_delta) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "yolo 模式: 检测/深度不同步或过期 (age=%.2fs, delta=%.2fs), 跳过",
                         age, frame_delta);
    return false;
  }

  const int u = static_cast<int>(std::lround(detection.bbox.center.position.x));
  // 抓取点取在框中心下方: 矿泉水瓶的框含瓶颈/瓶盖, 框中心落在肩部, 而夹爪要夹瓶身
  const int v = static_cast<int>(std::lround(
    detection.bbox.center.position.y + params_.grasp_offset_ratio * detection.bbox.size_y));
  const double score =
    detection.results.empty() ? 0.0 : detection.results.front().hypothesis.score;
  RCLCPP_INFO(this->get_logger(),
              "yolo 模式: 检测框=(%.1f, %.1f) %.1fx%.1f score=%.3f -> 抓取点像素=(%d, %d)",
              detection.bbox.center.position.x, detection.bbox.center.position.y,
              detection.bbox.size_x, detection.bbox.size_y, score, u, v);
  geometry_msgs::msg::PointStamped cam_point;
  bool published = projectPixelToCamera(*depth_img, u, v, cam_point);
  if (published) {
    // Identity follows the RGB detection frame. The matched depth image can
    // differ by up to max_detection_depth_delta; using its stamp would break
    // association with the frame that supplied the 2D bounding box.
    cam_point.header.stamp = detection_stamp;
    published = publishTarget(cam_point);
  }
  if (published) {
    processed_detection_stamp_ = detection_stamp.nanoseconds();
  }
  return published;
}

bool LocalizationNode::processPixelAt(int u, int v, const sensor_msgs::msg::Image & depth_img)
{
  geometry_msgs::msg::PointStamped cam_point;
  if (!projectPixelToCamera(depth_img, u, v, cam_point)) {
    return false;
  }
  return publishTarget(cam_point);
}

void LocalizationNode::colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat img;
    if (msg->encoding == "rgb8") {
      cv::Mat tmp(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()),
                  msg->step);
      cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgr8") {
      cv::Mat tmp(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()),
                  msg->step);
      img = tmp.clone();
    } else {
      return;  // 未知编码, 不显示
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_color_bgr_ = img;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "color 转换失败: %s",
                         e.what());
  }
}

void LocalizationNode::mouseCallbackCb(int event, int x, int y, int, void * userdata)
{
  static_cast<LocalizationNode *>(userdata)->onMouse(event, x, y);
}

void LocalizationNode::onMouse(int event, int x, int y)
{
  if (quit_requested_.load()) {
    // 已请求退出, 不再处理新的点击
    return;
  }
  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }
  sel_u_ = x;
  sel_v_ = y;
  has_selection_ = true;
  RCLCPP_INFO(this->get_logger(), "鼠标点击选点: pixel=(%d, %d)", x, y);

  sensor_msgs::msg::Image::SharedPtr depth;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    depth = latest_depth_;
  }
  if (!depth) {
    RCLCPP_WARN(this->get_logger(), "尚未收到深度帧, 无法读取该像素深度");
    return;
  }
  processPixelAt(x, y, *depth);
}

void LocalizationNode::displayLoop()
{
  while (rclcpp::ok() && !stop_display_.load()) {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!latest_color_bgr_.empty()) {
        frame = latest_color_bgr_.clone();
      }
    }
    if (!frame.empty()) {
      if (has_selection_.load()) {
        const cv::Point c(sel_u_.load(), sel_v_.load());
        cv::circle(frame, c, 15, cv::Scalar(0, 255, 255), 2);
        cv::line(frame, cv::Point(c.x - 25, c.y), cv::Point(c.x + 25, c.y),
                 cv::Scalar(0, 255, 255), 2);
        cv::line(frame, cv::Point(c.x, c.y - 25), cv::Point(c.x, c.y + 25),
                 cv::Scalar(0, 255, 255), 2);
      }
      cv::imshow(kWindowName, frame);
    }
    const int pressed_key = cv::waitKey(10);
    if (pressed_key == 'q' || pressed_key == 'Q') {
      quit_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "按下 '%c' 键, 关闭点击窗口并退出 localization_node",
                  pressed_key);
      // 触发全局 shutdown, 使主线程 exec.spin() 返回, 节点正常销毁退出
      rclcpp::shutdown();
      break;
    }
  }
  cv::destroyAllWindows();
}

bool LocalizationNode::projectPixelToCamera(const sensor_msgs::msg::Image & img, int u, int v,
                                            geometry_msgs::msg::PointStamped & out)
{
  if (u < 0 || v < 0 || u >= static_cast<int>(img.width) || v >= static_cast<int>(img.height)) {
    RCLCPP_ERROR(this->get_logger(), "像素 (%d,%d) 超出图像 %ux%u", u, v, img.width, img.height);
    return false;
  }

  sensor_msgs::msg::CameraInfo info;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    if (!latest_camera_info_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "尚未收到 camera_info");
      return false;
    }
    info = *latest_camera_info_;
  }
  const double fx = info.k[0];
  const double fy = info.k[4];
  const double cx = info.k[2];
  const double cy = info.k[5];
  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "camera_info K 无效");
    return false;
  }

  // 深度单位: 不假设 — 按编码显式区分。
  // RealSense 默认 16UC1 = uint16 毫米 (/1000 → 米); 32FC1 = float 米 (/1)。
  double z = 0.0;
  double raw = 0.0;
  const char * raw_unit = "";
  if (img.encoding == "16UC1") {
    if (img.step < img.width * 2) {
      RCLCPP_ERROR(this->get_logger(), "16UC1 图像 step=%u < width*2", img.step);
      return false;
    }
    uint16_t z_mm = 0;
    std::memcpy(&z_mm, &img.data[static_cast<size_t>(v) * img.step + static_cast<size_t>(u) * 2],
                sizeof(z_mm));
    z = static_cast<double>(z_mm) / 1000.0;
    raw = static_cast<double>(z_mm);
    raw_unit = "mm";
  } else if (img.encoding == "32FC1") {
    if (img.step < img.width * 4) {
      RCLCPP_ERROR(this->get_logger(), "32FC1 图像 step=%u < width*4", img.step);
      return false;
    }
    float z_m = 0.0f;
    std::memcpy(&z_m, &img.data[static_cast<size_t>(v) * img.step + static_cast<size_t>(u) * 4],
                sizeof(z_m));
    z = static_cast<double>(z_m);
    raw = z;
    raw_unit = "m";
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "未知深度编码 '%s' (支持 16UC1=毫米 / 32FC1=米)", img.encoding.c_str());
    return false;
  }

  if (!std::isfinite(z)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "深度非有限值 (NaN/Inf), 忽略该像素");
    return false;
  }
  if (z < params_.min_depth || z > params_.max_depth) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "深度 %.3f m 超出范围 [%.2f, %.2f], 忽略", z, params_.min_depth,
                         params_.max_depth);
    return false;
  }

  const double x = (static_cast<double>(u) - cx) * z / fx;
  const double y = (static_cast<double>(v) - cy) * z / fy;

  out.header.frame_id = params_.optical_frame;
  out.header.stamp = img.header.stamp;
  out.point.x = x;
  out.point.y = y;
  out.point.z = z;
  RCLCPP_INFO(this->get_logger(),
              "pixel=(%d, %d) raw_depth=%.0f %s depth_m=%.3f -> camera(%s) XYZ=(%.4f, %.4f, %.4f)",
              u, v, raw, raw_unit, z, params_.optical_frame.c_str(), x, y, z);
  return true;
}

bool LocalizationNode::publishTarget(const geometry_msgs::msg::PointStamped & cam_point)
{
  geometry_msgs::msg::PoseStamped target;
  try {
    const auto base_point =
      tf_buffer_.transform(cam_point, params_.target_frame, tf2::durationFromSec(0.5));
    target.header.frame_id = params_.target_frame;
    target.header.stamp = cam_point.header.stamp;
    target.pose.position = base_point.point;
    target.pose.orientation.w = params_.orient_w;
    target.pose.orientation.x = params_.orient_x;
    target.pose.orientation.y = params_.orient_y;
    target.pose.orientation.z = params_.orient_z;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "TF 转换失败 (%s→%s): %s", params_.optical_frame.c_str(),
                         params_.target_frame.c_str(), e.what());
    return false;
  }

  target_pose_pub_->publish(target);
  publishTargetMap(target, cam_point);
  publishMarker(target);
  const double cam_dist = std::sqrt(cam_point.point.x * cam_point.point.x +
                                    cam_point.point.y * cam_point.point.y +
                                    cam_point.point.z * cam_point.point.z);
  RCLCPP_INFO(this->get_logger(),
              "base(%s) XYZ=(%.4f, %.4f, %.4f) camera_to_target_distance=%.3f m -> /target_pose",
              params_.target_frame.c_str(), target.pose.position.x, target.pose.position.y,
              target.pose.position.z, cam_dist);
  return true;
}

void LocalizationNode::publishTargetMap(const geometry_msgs::msg::PoseStamped & base_target,
                                        const geometry_msgs::msg::PointStamped & cam_point)
{
  geometry_msgs::msg::PointStamped map_point;
  try {
    map_point = tf_buffer_.transform(cam_point, "map", tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & e) {
    // 无 map->base_link 时只 WARN 并跳过 /target_pose_map, 不影响 /target_pose
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "map->base_link 不可用, 跳过 /target_pose_map (不影响 /target_pose): %s",
                         e.what());
    return;
  }

  geometry_msgs::msg::PoseStamped map_target;
  map_target.header.frame_id = "map";
  map_target.header.stamp = base_target.header.stamp;
  map_target.pose.position = map_point.point;
  map_target.pose.orientation = base_target.pose.orientation;
  target_pose_map_pub_->publish(map_target);
  RCLCPP_INFO(this->get_logger(),
              "map(%s) XYZ=(%.4f, %.4f, %.4f) -> /target_pose_map (transient_local)",
              map_target.header.frame_id.c_str(), map_point.point.x, map_point.point.y,
              map_point.point.z);
}

void LocalizationNode::publishMarker(const geometry_msgs::msg::PoseStamped & target)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = params_.marker_frame;
  m.header.stamp = target.header.stamp;
  m.ns = "target";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position = target.pose.position;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = params_.marker_scale;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = rclcpp::Duration(0, 0);
  marker_pub_->publish(m);
}

}  // namespace scoutfetch_vision

// ==================== main ====================

#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<scoutfetch_vision::LocalizationNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
