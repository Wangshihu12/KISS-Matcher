#include "slam/loop_closure.h"

using namespace kiss_matcher;

LoopClosure::LoopClosure(const LoopClosureConfig &config, const rclcpp::Logger &logger)
    : config_(config), logger_(logger) {
  config_.matcher_config_ = kiss_matcher::KISSMatcherConfig(config_.voxel_res_, false);

  auto &gc          = config_.gicp_config_;
  gc.max_corr_dist_ = config_.voxel_res_ * gc.scale_factor_for_corr_dist_;

  src_cloud_.reset(new pcl::PointCloud<PointType>());
  tgt_cloud_.reset(new pcl::PointCloud<PointType>());
  coarse_aligned_.reset(new pcl::PointCloud<PointType>());
  aligned_.reset(new pcl::PointCloud<PointType>());
  debug_cloud_.reset(new pcl::PointCloud<PointType>());

  global_reg_handler_ = std::make_shared<kiss_matcher::KISSMatcher>(config_.matcher_config_);
  local_reg_handler_  = std::make_shared<small_gicp::RegistrationPCL<PointType, PointType>>();

  local_reg_handler_->setNumThreads(gc.num_threads_);
  local_reg_handler_->setCorrespondenceRandomness(gc.correspondence_randomness_);
  local_reg_handler_->setMaxCorrespondenceDistance(gc.max_corr_dist_);
  local_reg_handler_->setVoxelResolution(config.voxel_res_);
  local_reg_handler_->setRegistrationType("VGICP");  // "VGICP" or "GICP"
}

LoopClosure::~LoopClosure() {}

// NOTE(hlim): In outdoor scenes, loop closure sometimes fails due to Z-axis drift.
// To address this, we use the is_multilayer_env_ parameter.
// If true, full 3D distance (including Z) is considered for loop detection.
// If false, we ignore Z and compute distance on the XY plane only.
double LoopClosure::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2) {
  if (config_.is_multilayer_env_) {
    return (pose1.block<3, 1>(0, 3) - pose2.block<3, 1>(0, 3)).norm();
  } else {
    return (pose1.block<2, 1>(0, 3) - pose2.block<2, 1>(0, 3)).norm();
  }
}

LoopCandidates LoopClosure::getLoopCandidatesFromQuery(
    const PoseGraphNode &query_frame,
    const std::vector<PoseGraphNode> &keyframes) {
  LoopCandidates candidates;
  candidates.reserve(keyframes.size() / 100);  // heuristic: expect ~1% to be valid

  const auto &loop_det_radi      = config_.loop_detection_radius_;
  const auto &loop_det_tdiff_thr = config_.loop_detection_timediff_threshold_;

  for (size_t idx = 0; idx < keyframes.size() - 1; ++idx) {
    double dist = calculateDistance(keyframes[idx].pose_corrected_, query_frame.pose_corrected_);
    double time_diff = query_frame.timestamp_ - keyframes[idx].timestamp_;

    if (dist < loop_det_radi && time_diff > loop_det_tdiff_thr) {
      LoopCandidate c;
      c.idx_      = keyframes[idx].idx_;
      c.distance_ = dist;
      c.found_    = true;
      candidates.emplace_back(c);
    }
  }

  return candidates;
}

LoopCandidate LoopClosure::getClosestCandidate(const LoopCandidates &candidates) {
  if (candidates.empty()) return LoopCandidate();

  return *std::min_element(
      candidates.begin(), candidates.end(), [](const LoopCandidate &a, const LoopCandidate &b) {
        return a.distance_ < b.distance_;
      });
}

LoopIdxPairs LoopClosure::fetchClosestLoopCandidate(const PoseGraphNode &query_frame,
                                                    const std::vector<PoseGraphNode> &keyframes) {
  const auto &candidates = getLoopCandidatesFromQuery(query_frame, keyframes);
  if (candidates.empty()) {
    return {};
  }

  const auto &candidate = getClosestCandidate(candidates);
  // NOTE(hlim): While it outputs a single index pair,
  // for compatabiliy, I decided to use `LoopIdxPairs` instead of `LoopIdxPair`.
  LoopIdxPairs idx_pairs;
  idx_pairs.emplace_back(query_frame.idx_, candidate.idx_);
  return idx_pairs;
}

LoopIdxPairs LoopClosure::fetchLoopCandidates(const PoseGraphNode &query_frame,
                                              const std::vector<PoseGraphNode> &keyframes,
                                              const size_t num_max_candidates,
                                              const double reliable_window_sec) {
  // Case A.
  // If ICP was recently successful, it is highly likely that pose graph optimization (PGO)
  // has corrected the poses. As a result, the nearest neighbor becomes a much more reliable
  // loop candidate, and a single closest candidate is often sufficient.
  if (has_success_icp_time_) {
    auto now           = std::chrono::steady_clock::now();
    double elapsed_sec = std::chrono::duration<double>(now - last_success_icp_time_).count();

    if (elapsed_sec < reliable_window_sec) {
      if (config_.verbose_) {
        RCLCPP_INFO(logger_, "The nearest loop candidate is returned.");
      }
      return fetchClosestLoopCandidate(query_frame, keyframes);
    }
  }

  // Case B.
  // If no loop closure has occurred for a long time, the nearest neighbor is not always
  // a valid loop candidate due to possible drift or noise.
  // To mitigate this, we use random sampling to diversify the candidates.
  LoopCandidates candidates = getLoopCandidatesFromQuery(query_frame, keyframes);
  if (candidates.empty()) {
    return {};
  }

  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(candidates.begin(), candidates.end(), g);

  LoopIdxPairs idx_pairs;
  size_t num_selected = std::min(num_max_candidates, candidates.size());
  for (size_t i = 0; i < num_selected; ++i) {
    idx_pairs.emplace_back(query_frame.idx_, candidates[i].idx_);
  }

  return idx_pairs;
}

NodePair LoopClosure::setSrcAndTgtCloud(const std::vector<PoseGraphNode> &keyframes,
                                        const size_t src_idx,
                                        const size_t tgt_idx,
                                        const size_t num_submap_keyframes,
                                        const double voxel_res,
                                        const bool enable_global_registration) {
  const size_t submap_range = num_submap_keyframes / 2;
  const size_t num_approx   = keyframes[src_idx].scan_.size() * num_submap_keyframes;

  pcl::PointCloud<PointType> tgt_accum, src_accum;
  src_accum.reserve(num_approx);
  tgt_accum.reserve(num_approx);

  const bool build_submap = (num_submap_keyframes > 1);

  auto accumulateSubmap = [&](size_t center_idx, pcl::PointCloud<PointType> &accum) {
    const size_t start = (center_idx < submap_range) ? 0 : center_idx - submap_range;
    const size_t end   = std::min(center_idx + submap_range + 1, keyframes.size());
    for (size_t i = start; i < end; ++i) {
      accum += transformPcd(keyframes[i].scan_, keyframes[i].pose_corrected_);
    }
  };

  if (build_submap) {
    accumulateSubmap(src_idx, src_accum);
    accumulateSubmap(tgt_idx, tgt_accum);
  } else {
    src_accum = transformPcd(keyframes[src_idx].scan_, keyframes[src_idx].pose_corrected_);
    if (enable_global_registration) {
      tgt_accum = transformPcd(keyframes[tgt_idx].scan_, keyframes[tgt_idx].pose_corrected_);
    } else {
      // For ICP matching,
      // empirically scan-to-submap matching works better than scan-to-scan matching
      accumulateSubmap(tgt_idx, tgt_accum);
    }
  }
  return {*voxelize(src_accum, voxel_res), *voxelize(tgt_accum, voxel_res)};
}

void LoopClosure::setSrcAndTgtCloud(const pcl::PointCloud<PointType> &src_cloud,
                                    const pcl::PointCloud<PointType> &tgt_cloud) {
  *src_cloud_ = src_cloud;
  *tgt_cloud_ = tgt_cloud;
}

/**
 * [功能描述]：使用GICP算法执行局部点云配准（精配准）
 * 
 * 该函数实现基于GICP（广义迭代最近点）的局部配准：
 * - GICP是ICP算法的改进版本，考虑点云的局部表面结构
 * - 通常在粗配准之后使用，用于提高配准精度
 * - 通过迭代优化最小化点到平面的距离
 * 
 * 主要步骤：
 * - 准备点云数据并设置ICP算法的输入
 * - 执行迭代配准优化
 * - 计算重叠度（内点比例）
 * - 验证配准质量并更新成功时间戳
 * - 返回配准结果（变换矩阵、重叠度、有效性等）
 * 
 * @param src 源点云（待对齐的点云）
 * @param tgt 目标点云（参考点云）
 * @return RegOutput 配准输出结果，包含变换矩阵、重叠度、收敛性等信息
 */
RegOutput LoopClosure::icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &tgt) {
  // ============================================================
  // 步骤1: 初始化配准输出和清空缓存
  // ============================================================
  
  // 创建配准输出对象，用于存储配准结果
  RegOutput reg_output;
  // 清空对齐后的点云缓存（存储配准后的源点云）
  aligned_->clear();
  
  // ============================================================
  // 步骤2: 准备点云数据（在ICP之前合并子关键帧）
  // ============================================================
  
  // 创建源点云的智能指针（ICP算法要求使用智能指针）
  pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
  // 创建目标点云的智能指针
  pcl::PointCloud<PointType>::Ptr tgt_cloud(new pcl::PointCloud<PointType>());
  // 将输入的源点云复制到智能指针管理的对象中
  *src_cloud = src;
  // 将输入的目标点云复制到智能指针管理的对象中
  *tgt_cloud = tgt;
  
  // ============================================================
  // 步骤3: 配置GICP算法的输入数据
  // ============================================================
  
  // 设置目标点云（ICP会将源点云对齐到目标点云）
  local_reg_handler_->setInputTarget(tgt_cloud);
  // 设置源点云（待变换的点云）
  local_reg_handler_->setInputSource(src_cloud);

  // ============================================================
  // 步骤4: 执行GICP配准
  // ============================================================
  
  // 执行迭代配准，结果存储在aligned_中
  // GICP会迭代优化变换矩阵，使源点云与目标点云对齐
  local_reg_handler_->align(*aligned_);

  // ============================================================
  // 步骤5: 获取配准结果
  // ============================================================
  
  // 获取局部配准（GICP）的详细结果，包括内点数量、收敛信息等
  const auto &local_reg_result = local_reg_handler_->getRegistrationResult();

  // ============================================================
  // 步骤6: 计算点云重叠度（配准质量指标）
  // ============================================================
  
  // 计算重叠度 = (内点数量 / 源点云总点数) × 100%
  // 重叠度反映了两个点云的重叠程度，值越高表示配准质量越好
  double overlapness =
      static_cast<double>(local_reg_result.num_inliers) / src_cloud->size() * 100.0;
  // 将重叠度保存到输出结果中
  reg_output.overlapness_ = overlapness;

  // ============================================================
  // 步骤7: 获取配准变换矩阵
  // ============================================================
  
  // 注意：这是精配准相对于粗配准的增量变换（fine_T_coarse）
  // 获取GICP估计的最终变换矩阵（4x4）并转换为double类型
  reg_output.pose_ = local_reg_handler_->getFinalTransformation().cast<double>();
  
  // ============================================================
  // 步骤8: 验证配准质量
  // ============================================================
  
  // 检查重叠度是否超过阈值
  // 如果重叠度超过阈值，说明配准结果具有足够的重叠区域，配准可信
  if (overlapness > config_.gicp_config_.overlap_threshold_) {
    // 标记配准结果为有效
    reg_output.is_valid_     = true;
    // 标记算法已收敛
    reg_output.is_converged_ = true;

    // 记录最近一次ICP成功的时间戳（用于后续的回环候选策略）
    last_success_icp_time_ = std::chrono::steady_clock::now();
    // 标记已有成功的ICP记录
    has_success_icp_time_  = true;
  }
  
  // ============================================================
  // 步骤9: 输出配准结果日志
  // ============================================================
  
  // 如果启用详细输出模式，打印重叠度信息
  if (config_.verbose_) {
    // 判断重叠度是否达标
    if (overlapness > config_.gicp_config_.overlap_threshold_) {
      // 重叠度充足，用绿色输出成功信息
      RCLCPP_INFO(logger_,
                  "Overlapness: \033[1;32m%.2f%% > %.2f%%\033[0m",
                  overlapness,
                  config_.gicp_config_.overlap_threshold_);
    } else {
      // 重叠度不足，输出警告信息
      RCLCPP_WARN(logger_,
                  "Overlapness: %.2f%% < %.2f%%\033[0m",
                  overlapness,
                  config_.gicp_config_.overlap_threshold_);
    }
  }
  
  // 返回配准结果
  return reg_output;
}

/**
 * [功能描述]：执行从粗到精的两阶段点云配准
 * 
 * 该函数实现两阶段配准策略：
 * 1. 粗配准阶段：使用KISS-Matcher进行全局特征匹配，估计初始变换
 * 2. 精配准阶段：使用GICP进行局部迭代优化，提高配准精度
 * 
 * 主要步骤：
 * - 将点云数据转换为向量格式
 * - 使用全局配准器估计初始变换矩阵
 * - 应用粗配准变换
 * - 验证粗配准质量（内点数量）
 * - 如果粗配准成功，执行精配准
 * - 组合粗配准和精配准变换得到最终结果
 * 
 * @param src 源点云（待对齐的点云）
 * @param tgt 目标点云（参考点云）
 * @return RegOutput 配准输出结果，包含变换矩阵、内点数量、有效性标志等
 */
RegOutput LoopClosure::coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &tgt) {
  // ============================================================
  // 步骤1: 初始化配准输出和清空缓存
  // ============================================================
  
  // 创建配准输出对象，用于存储配准结果
  RegOutput reg_output;
  // 清空粗配准后的点云缓存
  coarse_aligned_->clear();

  // ============================================================
  // 步骤2: 数据格式转换
  // ============================================================
  
  // 将PCL格式的源点云转换为Eigen向量格式（用于KISS-Matcher）
  const auto &src_vec = convertCloudToVec(src);
  // 将PCL格式的目标点云转换为Eigen向量格式
  const auto &tgt_vec = convertCloudToVec(tgt);

  // ============================================================
  // 步骤3: 执行粗配准（全局配准）
  // ============================================================
  
  // 使用KISS-Matcher进行全局特征匹配，估计源点云到目标点云的变换
  // 该步骤通过特征描述符匹配来估计初始对齐，无需初始猜测
  const auto &solution = global_reg_handler_->estimate(src_vec, tgt_vec);

  // ============================================================
  // 步骤4: 构造粗配准变换矩阵
  // ============================================================
  
  // 创建4x4单位矩阵作为初始变换（齐次变换矩阵）
  Eigen::Matrix4d coarse_alignment      = Eigen::Matrix4d::Identity();
  // 填充左上角3x3旋转矩阵部分
  coarse_alignment.block<3, 3>(0, 0)    = solution.rotation.cast<double>();
  // 填充右上角3x1平移向量部分
  coarse_alignment.topRightCorner(3, 1) = solution.translation.cast<double>();

  // ============================================================
  // 步骤5: 应用粗配准变换
  // ============================================================
  
  // 使用粗配准变换矩阵对源点云进行变换，得到粗配准后的点云
  *coarse_aligned_ = transformPcd(src, coarse_alignment);

  // ============================================================
  // 步骤6: 评估粗配准质量
  // ============================================================
  
  // 获取粗配准过程中的内点数量（与模型一致的匹配点对数量）
  const size_t num_inliers      = global_reg_handler_->getNumFinalInliers();
  // 将内点数量保存到输出结果中
  reg_output.num_final_inliers_ = num_inliers;
  
  // 如果启用详细输出模式，打印内点数量信息
  if (config_.verbose_) {
    // 判断内点数量是否超过阈值
    if (num_inliers > config_.num_inliers_threshold_) {
      // 内点数量充足，用绿色输出成功信息
      RCLCPP_INFO(logger_,
                  "\033[1;32m# final inliers: %lu > %lu\033[0m",
                  num_inliers,
                  config_.num_inliers_threshold_);
    } else {
      // 内点数量不足，输出警告信息
      RCLCPP_WARN(
          logger_, "# final inliers: %lu < %lu", num_inliers, config_.num_inliers_threshold_);
    }
  }

  // ============================================================
  // 步骤7: 判断是否执行精配准
  // ============================================================
  
  // 注意：内点数量过少表明初始配准可能失败，此时执行精配准是无意义的
  // 检查全局配准是否有效或内点数量是否低于阈值
  if (!solution.valid || num_inliers < config_.num_inliers_threshold_) {
    // 粗配准失败，直接返回空结果（不执行精配准）
    return reg_output;
  } else {
    // ============================================================
    // 步骤8: 执行精配准（GICP）
    // ============================================================
    
    // 使用粗配准后的点云和目标点云进行GICP精配准
    // GICP会在粗配准基础上进行局部迭代优化，提高配准精度
    const auto &fine_output = icpAlignment(*coarse_aligned_, tgt);
    
    // 将精配准的结果复制到输出对象
    reg_output              = fine_output;
    
    // 组合精配准和粗配准的变换：最终变换 = 精配准变换 × 粗配准变换
    // fine_output.pose_ 是精配准阶段的增量变换（从coarse_aligned到tgt）
    // coarse_alignment 是粗配准阶段的变换（从src到coarse_aligned）
    reg_output.pose_        = fine_output.pose_ * coarse_alignment;

    // 调试用：可以使用此代码验证最终变换是否正确
    // 将源点云应用最终变换，用于可视化验证
    // *debug_cloud_        = transformPcd(src, reg_output.pose_);
  }
  
  // 返回最终配准结果
  return reg_output;
}

RegOutput LoopClosure::performLoopClosure(const PoseGraphNode &query_keyframe,
                                          const std::vector<PoseGraphNode> &keyframes) {
  const auto &loop_candidate = fetchClosestLoopCandidate(query_keyframe, keyframes);
  if (loop_candidate.empty()) {
    return RegOutput();
  }
  const auto &[query_idx, match_idx] = loop_candidate[0];
  return performLoopClosure(keyframes, query_idx, match_idx);
}

RegOutput LoopClosure::performLoopClosure(const std::vector<PoseGraphNode> &keyframes,
                                          const size_t query_idx,
                                          const size_t match_idx) {
  RegOutput reg_output;
  if (match_idx >= 0) {
    const auto &[src_cloud, tgt_cloud] = setSrcAndTgtCloud(keyframes,
                                                           query_idx,
                                                           match_idx,
                                                           config_.num_submap_keyframes_,
                                                           config_.voxel_res_,
                                                           config_.enable_global_registration_);
    // Only for visualization
    *src_cloud_ = src_cloud;
    *tgt_cloud_ = tgt_cloud;

    if (config_.enable_global_registration_) {
      RCLCPP_INFO(logger_,
                  "\033[1;35mExecute coarse-to-fine alignment: # src = %lu, # tgt = %lu\033[0m",
                  src_cloud.size(),
                  tgt_cloud.size());
      return coarseToFineAlignment(src_cloud, tgt_cloud);
    } else {
      RCLCPP_INFO(logger_,
                  "\033[1;35mExecute GICP: # src = %lu, # tgt = %lu\033[0m",
                  src_cloud.size(),
                  tgt_cloud.size());
      return icpAlignment(src_cloud, tgt_cloud);
    }
  } else {
    return reg_output;
  }
}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud() { return *src_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getTargetCloud() { return *tgt_cloud_; }

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud() { return *coarse_aligned_; }

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud() { return *aligned_; }

pcl::PointCloud<PointType> LoopClosure::getDebugCloud() { return *debug_cloud_; }
