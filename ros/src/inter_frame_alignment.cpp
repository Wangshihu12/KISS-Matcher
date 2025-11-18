#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "./tictoc.hpp"
#include "slam/loop_closure.h"
#include "slam/utils.hpp"

using namespace kiss_matcher;

class InterFrameAligner : public rclcpp::Node {
 public:
  /**
   * [功能描述]：帧间对齐器构造函数，初始化ROS2节点及相关参数
   * 
   * 主要初始化内容：
   * 1. 从ROS参数服务器加载配置参数（坐标系、频率、配准参数等）
   * 2. 配置回环检测和点云配准模块
   * 3. 创建TF坐标变换广播器
   * 4. 创建点云发布者（源点云、目标点云、配准结果）
   * 5. 创建定时器（配准、TF发布、可视化）
   * 6. 创建点云订阅者（源点云、目标点云）
   * 7. 初始化点云存储对象
   * 
   * @param options ROS2节点选项配置
   */
  explicit InterFrameAligner(const rclcpp::NodeOptions &options)
      : rclcpp::Node("inter_frame_aligner", options) {
    // ============================================================
    // 步骤1: 初始化配置对象和变量
    // ============================================================
    
    // 声明帧更新频率变量（Hz）
    double frame_update_hz;

    // 创建回环检测配置对象
    LoopClosureConfig lc_config;

    // 获取GICP（广义迭代最近点）配准算法的配置引用
    auto &gc = lc_config.gicp_config_;
    // 获取点云匹配器的配置引用
    auto &mc = lc_config.matcher_config_;

    // ============================================================
    // 步骤2: 从ROS参数服务器读取基础参数
    // ============================================================
    
    // 声明并获取源坐标系名称（如"base_link"）
    source_frame_   = declare_parameter<std::string>("source_frame", "");
    // 声明并获取目标坐标系名称（如"odom"）
    target_frame_   = declare_parameter<std::string>("target_frame", "");
    // 声明并获取世界坐标系名称，默认为"world"
    world_frame_    = declare_parameter<std::string>("world", "world");
    // 声明并获取帧对齐更新频率，默认为0.2Hz（每5秒更新一次）
    frame_update_hz = declare_parameter<double>("frame_update_hz", 0.2);
    // 声明并获取TF变换发布频率，默认为100Hz
    tf_hz_          = declare_parameter<double>("tf_hz", 100.0);

    // ============================================================
    // 步骤3: 配置回环检测模块的参数
    // ============================================================
    
    // 设置体素下采样分辨率（单位：米），用于点云降采样
    lc_config.voxel_res_ = declare_parameter<double>("voxel_resolution", 1.0);
    // 设置回环检测模块的详细输出模式
    lc_config.verbose_   = declare_parameter<bool>("loop.verbose", false);
    // 设置当前节点的详细输出模式
    verbose_             = declare_parameter<bool>("verbose", false);

    // ============================================================
    // 步骤4: 配置局部配准（GICP）的参数
    // ============================================================
    
    // 设置GICP算法使用的线程数，默认为8
    gc.num_threads_               = declare_parameter<int>("local_reg.num_threads", 8);
    // 设置对应点的随机选取数量，默认为20（用于加速计算）
    gc.correspondence_randomness_ = declare_parameter<int>("local_reg.correspondences_number", 20);
    // 设置GICP的最大迭代次数，默认为32
    gc.max_num_iter_              = declare_parameter<int>("local_reg.max_num_iter", 32);
    // 设置对应点距离的缩放因子，默认为5.0（用于确定对应点搜索半径）
    gc.scale_factor_for_corr_dist_ =
        declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
    // 设置点云重叠度阈值（百分比），默认为90%（用于判断配准是否有效）
    gc.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

    // ============================================================
    // 步骤5: 配置全局配准的参数
    // ============================================================
    
    // 设置是否启用全局配准（粗配准），默认关闭
    lc_config.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
    // 设置全局配准的内点数量阈值，默认为20（低于此值则认为配准失败）
    lc_config.num_inliers_threshold_ =
        declare_parameter<int>("global_reg.num_inliers_threshold", 20);

    // ============================================================
    // 步骤6: 配置ROS2消息通信的QoS策略
    // ============================================================
    
    // 创建QoS对象，队列深度为1（只保留最新消息）
    rclcpp::QoS qos(1);
    // 设置持久性策略为TRANSIENT_LOCAL（晚期订阅者可以接收到之前发布的消息）
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    // 设置可靠性策略为RELIABLE（保证消息传输可靠）
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    // ============================================================
    // 步骤7: 创建TF坐标变换广播器
    // ============================================================
    
    // 创建源坐标系的TF广播器（用于发布源点云到世界坐标系的变换）
    tf_source_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 创建目标坐标系的TF广播器（用于发布目标点云到世界坐标系的变换）
    tf_target_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ============================================================
    // 步骤8: 创建回环检测与配准模块
    // ============================================================
    
    // 创建回环检测模块实例，传入配置和日志器
    reg_module_ = std::make_shared<LoopClosure>(lc_config, this->get_logger());

    // ============================================================
    // 步骤9: 创建点云发布者
    // ============================================================
    
    // 创建源点云发布者，话题名为"reg/src"
    source_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/src", qos);
    // 创建目标点云发布者，话题名为"reg/tgt"
    target_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/tgt", qos);
    // 创建粗配准结果发布者，话题名为"reg/coarse_alignment"
    coarse_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/coarse_alignment", qos);
    // 创建精配准结果发布者，话题名为"reg/fine_alignment"
    fine_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("reg/fine_alignment", qos);

    // ============================================================
    // 步骤10: 创建定时器
    // ============================================================
    
    // 创建帧间对齐定时器，按照frame_update_hz频率执行performAlignment函数
    inter_alignment_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / frame_update_hz),
                                std::bind(&InterFrameAligner::performAlignment, this));

    // 创建TF发布定时器，按照tf_hz_频率执行publishTF函数（高频发布TF变换）
    tf_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / tf_hz_),
                                        std::bind(&InterFrameAligner::publishTF, this));

    // 创建点云可视化定时器，以20Hz频率执行visualizeClouds函数
    // 20Hz足够快，只要比完整配准过程更快即可
    cloud_vis_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                                std::bind(&InterFrameAligner::visualizeClouds, this));

    // ============================================================
    // 步骤11: 创建点云订阅者
    // ============================================================
    
    // 创建源点云订阅者，订阅"source"话题，接收到消息后调用callbackSource函数
    sub_source_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "source", qos, std::bind(&InterFrameAligner::callbackSource, this, std::placeholders::_1));
    // 创建目标点云订阅者，订阅"target"话题，接收到消息后调用callbackTarget函数
    sub_target_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "target", qos, std::bind(&InterFrameAligner::callbackTarget, this, std::placeholders::_1));

    // ============================================================
    // 步骤12: 初始化点云存储对象
    // ============================================================
    
    // 初始化源点云智能指针（使用PCL的PointType类型）
    source_cloud_.reset(new pcl::PointCloud<PointType>());
    // 初始化目标点云智能指针
    target_cloud_.reset(new pcl::PointCloud<PointType>());
  }

  void callbackSource(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Source map cloud has come!");
    }
    pcl::fromROSMsg(*msg, *source_cloud_);
    is_source_updated_ = true;
    source_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  void callbackTarget(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "Target map cloud has come!");
    }
    pcl::fromROSMsg(*msg, *target_cloud_);
    is_target_updated_ = true;
    target_timestamp_  = rclcpp::Time(msg->header.stamp);
  }

  /**
   * [功能描述]：执行源点云与目标点云的配准对齐
   * 
   * 该函数是配准流程的核心函数，由定时器定期调用。主要步骤：
   * 1. 检查源点云和目标点云是否都已更新
   * 2. 执行从粗到精的点云配准（先全局配准再局部优化）
   * 3. 记录配准耗时并输出日志
   * 4. 更新状态标志位
   * 5. 验证配准结果的有效性
   * 6. 保存配准得到的坐标变换并发布TF
   * 
   * @return 无返回值
   */
  void performAlignment() {
    // ============================================================
    // 步骤1: 检查点云数据是否准备就绪
    // ============================================================
    
    // 如果源点云或目标点云未更新，则直接返回（不执行配准）
    if (!is_source_updated_ || !is_target_updated_) return;

    // ============================================================
    // 步骤2: 执行点云配准并计时
    // ============================================================
    
    // 创建计时器对象，用于测量配准算法的执行时间
    kiss_matcher::TicToc timer;
    
    // 设置配准模块的源点云和目标点云（仅用于可视化保存）
    reg_module_->setSrcAndTgtCloud(*source_cloud_, *target_cloud_);
    
    // 执行从粗到精的配准流程：
    // 1. 粗配准：使用KISS-Matcher进行全局特征匹配，得到初始变换
    // 2. 精配准：使用GICP进行局部迭代优化，得到精确变换
    const auto &reg_output = reg_module_->coarseToFineAlignment(*source_cloud_, *target_cloud_);
    
    // 停止计时，获取配准总耗时（单位：毫秒）
    const auto t           = timer.toc();

    // ============================================================
    // 步骤3: 输出配准耗时信息
    // ============================================================
    
    // 输出配准算法的总执行时间（毫秒）
    RCLCPP_INFO(this->get_logger(), "Timing (msec) → Total: %.1f", t);

    // ============================================================
    // 步骤4: 更新状态标志位
    // ============================================================
    
    // 注意：无论配准结果是否精确，标志位都应该被更新
    // 这样可以确保节点继续接收和处理新的点云数据
    
    // 重置源点云更新标志（表示已处理当前源点云）
    is_source_updated_ = false;
    // 重置目标点云更新标志（表示已处理当前目标点云）
    is_target_updated_ = false;

    // 设置点云可视化更新标志（通知可视化线程有新结果需要发布）
    need_cloud_vis_update_ = true;

    // ============================================================
    // 步骤5: 配准结果验证
    // ============================================================
    
    // 定义一个极小值（用于浮点数比较的容差）
    const double eps = 1e-6;

    // 检查配准结果是否有效
    if (!reg_output.is_valid_) {
      // 如果配准被拒绝（内点数量不足或配准质量差），输出警告信息
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Alignment rejected. # of inliers: " << reg_output.num_final_inliers_);
    }

    // ============================================================
    // 步骤6: 保存配准结果并发布TF变换
    // ============================================================
    
    // 保存配准得到的从源坐标系到目标坐标系的变换矩阵（4x4）
    // target_T_source_ 表示将源点云变换到目标点云坐标系的SE(3)变换
    target_T_source_ = reg_output.pose_;
    
    // 立即发布TF变换，更新坐标系之间的关系
    publishTF();
  }

  geometry_msgs::msg::TransformStamped createTransformStamped(const Eigen::Matrix4d &from_T_to,
                                                              const std::string &from_frame,
                                                              const std::string &to_frame,
                                                              const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped transform_msg;

    // Extract rotation and translation
    Eigen::Matrix3d rot   = from_T_to.block<3, 3>(0, 0);
    Eigen::Vector3d trans = from_T_to.block<3, 1>(0, 3);
    Eigen::Quaterniond q(rot);

    // Fill message
    transform_msg.header.stamp            = stamp;
    transform_msg.header.frame_id         = from_frame;
    transform_msg.child_frame_id          = to_frame;
    transform_msg.transform.translation.x = trans.x();
    transform_msg.transform.translation.y = trans.y();
    transform_msg.transform.translation.z = trans.z();
    transform_msg.transform.rotation.x    = q.x();
    transform_msg.transform.rotation.y    = q.y();
    transform_msg.transform.rotation.z    = q.z();
    transform_msg.transform.rotation.w    = q.w();

    return transform_msg;
  }

  void publishTF() {
    static bool has_warned_waiting_for_clouds = false;
    if ((!source_timestamp_.has_value()) || (!target_timestamp_.has_value())) {
      if (!has_warned_waiting_for_clouds) {
        RCLCPP_WARN(this->get_logger(), "Waiting for map clouds...");
        has_warned_waiting_for_clouds = true;
      }
      return;
    }

    // NOTE(hlim): To visualize real-time topics, such as current scans,
    // we have to incrementally increase the timestamp
    rclcpp::Time now = this->get_clock()->now();
    if (!source_timestamp_base_.has_value() || *source_timestamp_ != *source_timestamp_base_) {
      source_timestamp_base_ = source_timestamp_;
      last_real_time_        = now;
    }
    if (!target_timestamp_base_.has_value() || *target_timestamp_ != *target_timestamp_base_) {
      target_timestamp_base_ = target_timestamp_;
      last_real_time_        = now;
    }

    rclcpp::Duration elapsed = now - last_real_time_;
    if (elapsed.nanoseconds() < 0) {
      elapsed = rclcpp::Duration::from_nanoseconds(0);
    }

    rclcpp::Time source_time = *source_timestamp_base_ + elapsed;
    rclcpp::Time target_time = *target_timestamp_base_ + elapsed;

    const auto &world_to_source_msg =
        createTransformStamped(target_T_source_, world_frame_, source_frame_, source_time);
    const auto &world_to_target_msg = createTransformStamped(
        Eigen::Matrix4d::Identity(), world_frame_, target_frame_, target_time);

    tf_source_broadcaster_->sendTransform(world_to_source_msg);
    tf_target_broadcaster_->sendTransform(world_to_target_msg);
  }

  void visualizeClouds() {
    if (!need_cloud_vis_update_) {
      return;
    }
    source_pub_->publish(toROSMsg(std::move(reg_module_->getSourceCloud()), world_frame_));
    target_pub_->publish(toROSMsg(std::move(reg_module_->getTargetCloud()), world_frame_));
    fine_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getFinalAlignedCloud()), world_frame_));
    coarse_aligned_pub_->publish(
        toROSMsg(std::move(reg_module_->getCoarseAlignedCloud()), world_frame_));

    RCLCPP_INFO(this->get_logger(), "Clouds published!");
    need_cloud_vis_update_ = false;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_source_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_target_;

  std::string source_frame_;
  std::string target_frame_;
  std::string world_frame_;

  pcl::PointCloud<PointType>::Ptr source_cloud_;
  pcl::PointCloud<PointType>::Ptr target_cloud_;

  std::optional<rclcpp::Time> source_timestamp_;
  std::optional<rclcpp::Time> source_timestamp_base_;
  std::optional<rclcpp::Time> target_timestamp_;
  std::optional<rclcpp::Time> target_timestamp_base_;
  rclcpp::Time last_real_time_;

  bool verbose_ = false;

  bool is_source_updated_ = false;
  bool is_target_updated_ = false;

  bool need_cloud_vis_update_ = false;

  std::shared_ptr<kiss_matcher::LoopClosure> reg_module_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_source_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_target_broadcaster_;

  kiss_matcher::TicToc timer_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr coarse_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fine_aligned_pub_;

  rclcpp::TimerBase::SharedPtr inter_alignment_timer_;
  rclcpp::TimerBase::SharedPtr cloud_vis_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  double tf_hz_;

  Eigen::Matrix4d target_T_source_ = Eigen::Matrix4d::Identity();
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<InterFrameAligner>(options);

  // To allow timer callbacks to run concurrently using multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
