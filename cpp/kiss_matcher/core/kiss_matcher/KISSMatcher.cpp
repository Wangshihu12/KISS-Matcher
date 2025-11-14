/**
 * Copyright 2024, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Hyungtae Lim, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 */

#include <kiss_matcher/KISSMatcher.hpp>

namespace kiss_matcher {
KISSMatcher::KISSMatcher(const float &voxel_size) { config_ = KISSMatcherConfig(voxel_size); }

KISSMatcher::KISSMatcher(const KISSMatcherConfig &config) {
  config_ = config;
  reset();
}

void KISSMatcher::reset() {
  faster_pfh_ = std::make_unique<FasterPFH>(
      config_.normal_radius_, config_.fpfh_radius_, config_.thr_linearity_);
  robin_matching_ = std::make_unique<ROBINMatching>(
      config_.robin_noise_bound_, config_.num_max_corr_, config_.tuple_scale_);

  resetSolver();
}

void KISSMatcher::resetSolver() {
  // NOTE(hlim) Please turn on `use_quatro_`
  // when the pitch and roll angles are not dominant in the rotation
  kiss_matcher::RobustRegistrationSolver::Params params;
  params.noise_bound = config_.solver_noise_bound_;

  if (config_.use_quatro_) {
    params.rotation_estimation_algorithm =
        kiss_matcher::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::QUATRO;
  } else {
    params.rotation_estimation_algorithm =
        kiss_matcher::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  }

  solver_ = std::make_unique<RobustRegistrationSolver>(params);
}

/**
 * [功能描述]：执行源点云与目标点云之间的特征匹配
 * 该函数完成点云预处理、特征提取、特征匹配的完整流程，并记录各阶段耗时
 * @param src：源点云，类型为 std::vector<Eigen::Vector3f>，包含待匹配的三维点集
 * @param tgt：目标点云，类型为 std::vector<Eigen::Vector3f>，包含目标参考三维点集
 * @return KeypointPair：匹配的关键点对，包含源点云和目标点云中相互对应的关键点
 */
kiss_matcher::KeypointPair KISSMatcher::match(const std::vector<Eigen::Vector3f> &src,
                                              const std::vector<Eigen::Vector3f> &tgt) {
  // 清空之前的匹配结果和中间数据
  clear();
  
  // 定义Lambda函数用于点云预处理
  // 根据配置决定是否进行体素网格下采样以减少点云数量
  auto processInput = [&](const std::vector<Eigen::Vector3f> &input_cloud) {
    if (config_.use_voxel_sampling_) {
      // 使用体素网格采样降低点云密度，提高后续处理效率
      return VoxelgridSampling(input_cloud, config_.voxel_size_);
    }
    // 不进行下采样，直接返回原始点云
    return input_cloud;
  };

  // 记录初始时间戳，用于计算预处理阶段耗时
  auto t_init = std::chrono::high_resolution_clock::now();

  // 处理源点云和目标点云（可选的体素采样）
  // 使用std::move避免不必要的拷贝，提高效率
  src_processed_ = std::move(processInput(src));
  tgt_processed_ = std::move(processInput(tgt));

  // 记录预处理完成时间戳
  auto t_process = std::chrono::high_resolution_clock::now();

  // ==== 源点云特征提取 ====
  // 设置FasterPFH算法的输入点云为已处理的源点云
  faster_pfh_->setInputCloud(src_processed_);
  // 计算源点云的FPFH特征描述符
  // 注意：某些异常点会被过滤，因此 src_keypoints_ 的数量 <= src_processed_ 的数量
  faster_pfh_->ComputeFeature(src_keypoints_, src_descriptors_);

  // ==== 目标点云特征提取 ====
  // 设置FasterPFH算法的输入点云为已处理的目标点云
  faster_pfh_->setInputCloud(tgt_processed_);
  // 计算目标点云的FPFH特征描述符
  // 注意：某些异常点会被过滤，因此 tgt_keypoints_ 的数量 <= tgt_processed_ 的数量
  faster_pfh_->ComputeFeature(tgt_keypoints_, tgt_descriptors_);

  // 记录特征提取完成时间戳
  auto t_mid = std::chrono::high_resolution_clock::now();

  // ==== 建立点云对应关系 ====
  // 使用ROBIN算法基于特征描述符建立源点云和目标点云之间的对应关系
  // corr存储匹配结果，每个元素是一个包含源点索引和目标点索引的pair
  const auto &corr = robin_matching_->establishCorrespondences(src_keypoints_,
                                                               tgt_keypoints_,
                                                               src_descriptors_,
                                                               tgt_descriptors_,
                                                               config_.robin_mode_,
                                                               config_.tuple_scale_,
                                                               config_.use_ratio_test_);

  // 调整匹配点容器大小以容纳所有对应点对
  src_matched_.resize(corr.size());
  tgt_matched_.resize(corr.size());

  // 遍历所有对应关系，提取实际的匹配点坐标
  for (size_t i = 0; i < corr.size(); ++i) {
    // 从对应关系中获取源点云中的关键点索引
    auto src_idx    = std::get<0>(corr[i]);
    // 从对应关系中获取目标点云中的关键点索引
    auto dst_idx    = std::get<1>(corr[i]);
    // 根据索引提取源点云中的匹配点坐标
    src_matched_[i] = src_keypoints_[src_idx];
    // 根据索引提取目标点云中的匹配点坐标
    tgt_matched_[i] = tgt_keypoints_[dst_idx];
  }
  
  // 记录匹配完成时间戳
  auto t_end = std::chrono::high_resolution_clock::now();

  // ==== 计算各阶段耗时 ====
  // 预处理阶段耗时（体素采样）
  processing_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_process - t_init).count();
  // 特征提取阶段耗时（FPFH计算）
  extraction_time_ =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_mid - t_process).count();
  // 特征匹配阶段耗时（ROBIN对应关系建立）
  matching_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_mid).count();

  // 返回匹配的关键点对
  return {src_matched_, tgt_matched_};
}

kiss_matcher::KeypointPair KISSMatcher::match(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src,
                                              const Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt) {
  std::vector<Eigen::Vector3f> src_vec(src.cols());
  std::vector<Eigen::Vector3f> tgt_vec(tgt.cols());

  for (ssize_t i = 0; i < src.cols(); ++i) {
    src_vec[i] = src.col(i).cast<float>();
  }
  for (ssize_t i = 0; i < tgt.cols(); ++i) {
    tgt_vec[i] = tgt.col(i).cast<float>();
  }

  return match(src_vec, tgt_vec);
}

/**
 * [功能描述]：估计源点云到目标点云的配准变换
 * 该函数首先进行点云匹配，然后基于匹配结果求解最优的刚性变换（旋转和平移）
 * @param src：源点云，类型为 std::vector<Eigen::Vector3f>，包含待配准的三维点集
 * @param tgt：目标点云，类型为 std::vector<Eigen::Vector3f>，包含目标参考三维点集
 * @return RegistrationSolution：配准结果，包含旋转矩阵、平移向量和内点信息
 */
kiss_matcher::RegistrationSolution KISSMatcher::estimate(const std::vector<Eigen::Vector3f> &src,
                                                         const std::vector<Eigen::Vector3f> &tgt) {
  // 调用match函数进行特征匹配，获取源点云和目标点云中的匹配点对
  // 使用结构化绑定直接解构返回的KeypointPair
  const auto &[src_matched, tgt_matched] = match(src, tgt);
  
  // 获取匹配点对的数量
  size_t M                               = src_matched.size();

  // 声明用于存储匹配点的Eigen矩阵
  // 矩阵维度：3行（x, y, z坐标），M列（每列对应一个匹配点）
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched_eigen;
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched_eigen;

  // 调整矩阵大小为3行M列，以容纳所有匹配点
  src_matched_eigen.resize(3, M);
  tgt_matched_eigen.resize(3, M);
  
  // 遍历所有匹配点对，将Vector3f格式转换为Matrix<double, 3, Dynamic>格式
  for (size_t m = 0; m < M; ++m) {
    // 将第m个源匹配点从float精度转换为double精度，并存入矩阵的第m列
    src_matched_eigen.col(m) << src_matched[m].cast<double>();
    // 将第m个目标匹配点从float精度转换为double精度，并存入矩阵的第m列
    tgt_matched_eigen.col(m) << tgt_matched[m].cast<double>();
  }
  
  // 调用solve函数，基于匹配点对求解最优的刚性变换
  return solve(src_matched_eigen, tgt_matched_eigen);
}

/**
 * [功能描述]：基于匹配点对求解刚性配准变换
 * 该函数使用鲁棒求解器计算最优的旋转矩阵和平移向量，将源点云对齐到目标点云
 * @param src_matched：源点云中的匹配点，类型为 Eigen::Matrix<double, 3, Eigen::Dynamic>
 *                     矩阵维度为 3×N（3行代表x/y/z坐标，N列代表匹配点数量）
 * @param tgt_matched：目标点云中的匹配点，类型为 Eigen::Matrix<double, 3, Eigen::Dynamic>
 *                     矩阵维度为 3×N（3行代表x/y/z坐标，N列代表匹配点数量）
 * @return RegistrationSolution：配准结果，包含旋转矩阵R、平移向量t和内点信息
 */
RegistrationSolution KISSMatcher::solve(
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &src_matched,
    const Eigen::Matrix<double, 3, Eigen::Dynamic> &tgt_matched) {
  // 检查匹配点对数量是否足够
  // 如果匹配点对少于2个，无法求解有效的刚性变换
  // 直接返回默认的无效解（单位矩阵）
  if (src_matched.cols() < 2) {
    return solver_->getSolution();
  }

  // 重置求解器状态，确保每次求解都是从干净的状态开始
  // 根据配置初始化旋转估计算法（QUATRO或GNC_TLS）
  resetSolver();
  
  // 记录求解开始时间戳
  std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
  
  // 调用鲁棒配准求解器求解最优变换
  // 求解器会自动处理外点，估计旋转矩阵R和平移向量t
  solver_->solve(src_matched, tgt_matched);
  
  // 记录求解结束时间戳
  std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
  
  // 计算求解器耗时（单位：秒）
  solver_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();

  // 返回配准求解结果，包含旋转矩阵、平移向量和内点信息
  return solver_->getSolution();
}

RegistrationSolution KISSMatcher::pruneAndSolve(const std::vector<Eigen::Vector3f> &src_matched,
                                                const std::vector<Eigen::Vector3f> &tgt_matched) {
  std::vector<std::pair<int, int>> corres, corres_out;
  for (size_t i = 0; i < src_matched.size(); ++i) {
    corres.emplace_back(i, i);
  }
  const auto &pruned_indices =
      robin_matching_->applyOutlierPruning(src_matched, tgt_matched, "max_core");
  size_t num_pruned_corr = pruned_indices.size();

  Eigen::Matrix<double, 3, Eigen::Dynamic> src_eigen(3, num_pruned_corr);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_eigen(3, num_pruned_corr);

  for (size_t i = 0; i < num_pruned_corr; ++i) {
    src_eigen.col(i) = src_matched[pruned_indices[i]].cast<double>();
    tgt_eigen.col(i) = tgt_matched[pruned_indices[i]].cast<double>();
  }
  return solve(src_eigen, tgt_eigen);
}

double KISSMatcher::getProcessingTime() { return processing_time_; }

double KISSMatcher::getExtractionTime() { return extraction_time_; }

double KISSMatcher::getRejectionTime() { return robin_matching_->getRejectionTime(); }

double KISSMatcher::getMatchingTime() { return matching_time_; }

double KISSMatcher::getSolverTime() { return solver_time_; }

void KISSMatcher::print() {
  const double t_p = getProcessingTime();
  const double t_e = getExtractionTime();
  const double t_r = getRejectionTime();
  const double t_m = getMatchingTime();
  const double t_s = getSolverTime();

  std::cout << "============== Time =============="
            << "\n";
  std::cout << "Voxelization: " << t_p << " sec\n";
  std::cout << "Extraction  : " << t_e << " sec\n";
  std::cout << "Pruning     : " << t_r << " sec\n";
  std::cout << "Matching    : " << t_m << " sec\n";
  std::cout << "Solving     : " << t_s << " sec\n";
  std::cout << "----------------------------------"
            << "\n";
  std::cout << "\033[1;32mTotal     : " << t_p + t_e + t_r + t_m + t_s << " sec\033[0m\n";
  std::cout << "====== # of correspondences ======"
            << "\n";
  std::cout << "# initial pairs : " << robin_matching_->getNumInitialCorrespondences() << "\n";
  std::cout << "# pruned pairs  : " << robin_matching_->getNumPrunedCorrespondences() << "\n";
  std::cout << "----------------------------------"
            << "\n";
  std::cout << "\033[1;36m# rot inliers   : " << solver_->getRotationInliers().size() << "\n";
  std::cout << "# trans inliers : " << solver_->getTranslationInliers().size() << "\033[0m\n";
  std::cout << "=================================="
            << "\n";
}
}  // namespace kiss_matcher
