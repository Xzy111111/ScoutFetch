// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>

namespace scoutfetch_vision
{

class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationNode() override;

private:
  struct Params
  {
    int pixel_u = 320;
    int pixel_v = 240;
    // 像素来源:
    //   "yaml"  = 固定像素 (只对标定时的机器人位姿有效)
    //   "click" = OpenCV 窗口鼠标左键点击选点
    //   "yolo"  = 取 /detections (vision_msgs/Detection2DArray) 中置信度最高的目标框中心,
    //             与机器人位姿无关, 检测不到就不发布
    std::string pixel_source = "yaml";
    std::string detection_topic = "/detections";
    // 超过该时长未更新的检测视为失效, 不再使用 (避免目标离开视野后继续用旧像素)
    double detection_timeout = 1.0;
    // Reject RGB boxes applied to depth from a different viewpoint during rotation.
    double max_detection_depth_delta = 0.12;
    // 抓取点取在检测框中心下方 grasp_offset_ratio * 框高 处。
    // 矿泉水瓶的检测框含瓶颈/瓶盖, 框中心落在肩部; 夹爪要夹的是瓶身, 所以往下偏。
    double grasp_offset_ratio = 0.15;
    std::string color_topic = "/camera/camera/color/image_raw";
    std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";
    std::string depth_info_topic = "/camera/camera/aligned_depth_to_color/camera_info";
    std::string optical_frame = "camera_color_optical_frame";
    std::string target_frame = "base_link";
    double min_depth = 0.10;
    double max_depth = 5.0;
    // 第一版固定目标姿态 = scoutfetch_grasp 已验证的抓取姿态 Ry(110°)
    double orient_w = 0.573932;
    double orient_x = 0.0;
    double orient_y = 0.818903;
    double orient_z = 0.0;
    double publish_rate = 10.0;
    double marker_scale = 0.03;
    std::string marker_frame = "base_link";
  };
  Params params_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  // 世界系稳定目标 (map), transient_local + keep_last(1), 供后启动的 scoutfetch_mission 取用
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::mutex info_mutex_;
  std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::chrono::steady_clock::time_point last_publish_;
  std::chrono::milliseconds publish_interval_;

  // ---- click 模式 (OpenCV 鼠标选点) ----
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  std::mutex frame_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> depth_history_;
  cv::Mat latest_color_bgr_;
  std::atomic<bool> has_selection_{false};
  std::atomic<int> sel_u_{0};
  std::atomic<int> sel_v_{0};
  std::thread display_thread_;
  std::atomic<bool> stop_display_{false};
  // 用户在窗口按下 q/Q 请求退出: 不再处理新点击, 并触发 ROS2 优雅关闭
  std::atomic<bool> quit_requested_{false};

  // ---- yolo 模式: 最近一次检测 (订阅 /detections, 取置信度最高者) ----
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  std::mutex detection_mutex_;
  std::optional<vision_msgs::msg::Detection2D> latest_detection_;
  rclcpp::Time latest_detection_stamp_;
  std::optional<int64_t> processed_detection_stamp_;

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool projectPixelToCamera(const sensor_msgs::msg::Image & img, int u, int v,
                            geometry_msgs::msg::PointStamped & out);
  bool publishTarget(const geometry_msgs::msg::PointStamped & cam_point);
  // /target_pose_map: map 系稳定目标; 无 map->base_link 时只 WARN 跳过, 不影响 /target_pose
  void publishTargetMap(const geometry_msgs::msg::PoseStamped & base_target,
                        const geometry_msgs::msg::PointStamped & cam_point);
  void publishMarker(const geometry_msgs::msg::PoseStamped & target);
  bool processPixelAt(int u, int v, const sensor_msgs::msg::Image & depth_img);
  // yolo 模式: 用最新检测框中心作为像素; 没有检测 (或检测已过期) 时不发布,
  // 不得发一个猜出来的坐标 —— task_fsm 会把收到的任何 /target_pose_map 冻结成任务目标
  bool processDetectedTarget();
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void onMouse(int event, int x, int y);
  void displayLoop();
  // OpenCV setMouseCallback 需要 C 兼容函数指针; 静态成员可访问私有 onMouse
  static void mouseCallbackCb(int event, int x, int y, int flags, void * userdata);
};

}  // namespace scoutfetch_vision
