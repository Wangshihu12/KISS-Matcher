#include <filesystem>

#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "quatro/quatro_utils.h"

bool readBin(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
  if (!ifs) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return false;
  }

  std::streamsize points_bytes = ifs.tellg();
  size_t num_points            = points_bytes / (sizeof(Eigen::Vector4f));

  ifs.seekg(0, std::ios::beg);
  std::vector<Eigen::Vector4f> points(num_points);
  ifs.read(reinterpret_cast<char*>(points.data()), sizeof(Eigen::Vector4f) * num_points);

  cloud.clear();
  cloud.reserve(num_points);

  pcl::PointXYZ point;
  for (auto& pt : points) {
    point.x = pt(0);
    point.y = pt(1);
    point.z = pt(2);
    cloud.emplace_back(point);
  }

  return true;
}

bool loadPointCloud(const std::string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::string extension = std::filesystem::path(filepath).extension().string();

  if (extension == ".pcd") {
    return pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) >= 0;
  } else if (extension == ".ply") {
    return pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *cloud) >= 0;
  } else if (extension == ".bin") {
    return readBin(filepath, *cloud);
  } else {
    std::cerr << "Unsupported file format: " << extension << std::endl;
    return false;
  }
}

std::vector<Eigen::Vector3f> convertCloudToVec(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud.size());
  for (const auto& pt : cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}

/**
 * [功能描述]：KISS-Matcher点云配准示例程序主函数
 * 该程序加载源点云和目标点云，执行配准算法，并可视化结果
 * @param argc：命令行参数数量
 * @param argv：命令行参数数组
 *             argv[1]：源点云文件路径
 *             argv[2]：目标点云文件路径
 *             argv[3]：分辨率参数（用于体素网格下采样）
 *             argv[4]：偏航角增强角度（可选，单位：度）
 * @return 成功返回0，失败返回-1
 */
int main(int argc, char** argv) {
  // ==== 命令行参数检查 ====
  // 至少需要4个参数：程序名、源点云文件、目标点云文件、分辨率
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <src_pcd_file> <tgt_pcd_file> <resolution> <yaw_aug_angle>" << std::endl;
    return -1;
  }
  
  // 创建PCL点云智能指针，用于存储源点云和目标点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl(new pcl::PointCloud<pcl::PointXYZ>);

  // ==== 解析命令行参数 ====
  // 源点云文件路径
  const std::string src_path = argv[1];
  // 目标点云文件路径
  const std::string tgt_path = argv[2];
  // 分辨率参数，用于体素网格下采样和特征提取
  const float resolution     = std::stof(argv[3]);

  // ==== 偏航角变换设置 ====
  // 初始化为单位矩阵（4×4齐次变换矩阵）
  Eigen::Matrix4f yaw_transform = Eigen::Matrix4f::Identity();
  if (argc > 4) {
    // 获取用户指定的偏航角（单位：度）
    float yaw_aug_angle = std::stof(argv[4]);
    // 将角度转换为弧度
    float yaw_rad       = yaw_aug_angle * M_PI / 180.0f;

    // 构造绕Z轴旋转的旋转矩阵
    // | cos(θ) -sin(θ)  0  0 |
    // | sin(θ)  cos(θ)  0  0 |
    // |   0       0     1  0 |
    // |   0       0     0  1 |
    yaw_transform(0, 0) = std::cos(yaw_rad);
    yaw_transform(0, 1) = -std::sin(yaw_rad);
    yaw_transform(1, 0) = std::sin(yaw_rad);
    yaw_transform(1, 1) = std::cos(yaw_rad);
  }

  // 输出输入文件路径信息
  std::cout << "Source input: " << src_path << "\n";
  std::cout << "Target input: " << tgt_path << "\n";

  // ==== 加载点云文件 ====
  // 支持.pcd、.ply、.bin格式的点云文件
  if (!loadPointCloud(src_path, src_pcl) || !loadPointCloud(tgt_path, tgt_pcl)) {
    std::cerr << "Error loading point cloud files." << std::endl;
    return -1;
  }

  // ==== 移除无效点（NaN点） ====
  // 存储有效点的索引
  std::vector<int> src_indices;
  std::vector<int> tgt_indices;
  // 从源点云中移除NaN（非数值）点
  pcl::removeNaNFromPointCloud(*src_pcl, *src_pcl, src_indices);
  // 从目标点云中移除NaN点
  pcl::removeNaNFromPointCloud(*tgt_pcl, *tgt_pcl, tgt_indices);

  // ==== 对源点云应用偏航角变换 ====
  // 创建新的点云用于存储旋转后的源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_src_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  // 应用偏航角变换
  pcl::transformPointCloud(*src_pcl, *rotated_src_pcl, yaw_transform);
  // 更新源点云指针指向旋转后的点云
  src_pcl = rotated_src_pcl;

  // ==== 将PCL点云转换为Eigen向量格式 ====
  // 转换源点云为std::vector<Eigen::Vector3f>格式，供KISS-Matcher使用
  const auto& src_vec = convertCloudToVec(*src_pcl);
  // 转换目标点云为std::vector<Eigen::Vector3f>格式
  const auto& tgt_vec = convertCloudToVec(*tgt_pcl);

  // 输出加载完成信息（绿色文本）
  std::cout << "\033[1;32mLoad complete!\033[0m\n";

  // ==== 配置KISS-Matcher算法参数 ====
  // 创建配置对象，使用指定的分辨率参数
  kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig(resolution);
  
  // 注意：两个重要的性能优化参数
  // 1. config.use_quatro_（默认值：false）
  //    如果旋转主要围绕偏航轴（Z轴），应设置为true：
  //    config.use_quatro_ = true;
  //    否则，默认模式会激活基于SO(3)的GNC求解器
  //    例如，对于VBR-Collosseo数据集，应设置为false
  //
  // 2. config.use_ratio_test_（默认值：true）
  //    对于扫描级别的点云，use_ratio_test_的影响不大
  //    将use_ratio_test_设置为false可以略微加速
  //    如果处理扫描级别或回环检测场景，设置为false可以提升推理速度
  //    config.use_ratio_test_ = false;
  
  // 创建KISS-Matcher匹配器对象
  kiss_matcher::KISSMatcher matcher(config);

  // ==== 执行点云配准 ====
  // 估计从源点云到目标点云的刚性变换（旋转+平移）
  const auto solution = matcher.estimate(src_vec, tgt_vec);

  // ==== 准备可视化数据 ====
  // 复制源点云用于可视化
  pcl::PointCloud<pcl::PointXYZ> src_viz = *src_pcl;
  // 复制目标点云用于可视化
  pcl::PointCloud<pcl::PointXYZ> tgt_viz = *tgt_pcl;
  // 创建用于存储配准后源点云的变量
  pcl::PointCloud<pcl::PointXYZ> est_viz;

  // ==== 将配准结果转换为齐次变换矩阵 ====
  // 初始化为4×4单位矩阵
  Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
  // 将旋转矩阵（3×3）赋值到左上角块，并转换为float精度
  solution_eigen.block<3, 3>(0, 0)    = solution.rotation.cast<float>();
  // 将平移向量（3×1）赋值到右上角，并转换为float精度
  solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

  // 打印配准统计信息（耗时、对应点数量等）
  matcher.print();

  // ==== 获取内点数量用于判断配准是否成功 ====
  // 旋转内点数量：满足旋转约束的对应点数
  size_t num_rot_inliers   = matcher.getNumRotationInliers();
  // 最终内点数量：同时满足旋转和平移约束的对应点数
  size_t num_final_inliers = matcher.getNumFinalInliers();

  // 注意：通过检查最终内点数量可以判断配准是否成功
  // 阈值越大，判断越保守
  // 参见 https://github.com/MIT-SPARK/KISS-Matcher/issues/24
  size_t thres_num_inliers = 5;
  if (num_final_inliers < thres_num_inliers) {
    // 最终内点数量少于阈值，配准可能失败（黄色文本）
    std::cout << "\033[1;33m=> Registration might have failed :(\033[0m\n";
  } else {
    // 最终内点数量足够，配准可能成功（绿色文本）
    std::cout << "\033[1;32m=> Registration likely succeeded XD\033[0m\n";
  }

  // 输出配准变换矩阵
  std::cout << solution_eigen << std::endl;
  std::cout << "=====================================" << std::endl;

  // ============================================================
  // 保存变换后的源点云
  // ============================================================
  // 创建新的点云用于存储变换后的源点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr est_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // 将配准变换应用到源点云，得到配准后的点云
  pcl::transformPointCloud(*src_pcl, *est_cloud, solution_eigen);
  
  // 解析源文件路径
  std::filesystem::path src_file_path(src_path);
  // 构造输出文件名：在源文件名后添加"_warped"后缀
  std::string warped_pcd_filename =
      src_file_path.parent_path().string() + "/" + src_file_path.stem().string() + "_warped.pcd";
  // 将变换后的点云保存为ASCII格式的PCD文件
  pcl::io::savePCDFileASCII(warped_pcd_filename, *est_cloud);
  std::cout << "Saved transformed source point cloud to: " << warped_pcd_filename << std::endl;

  // ============================================================
  // 可视化配准结果
  // ============================================================
  // 将配准变换应用到源点云用于可视化
  pcl::transformPointCloud(src_viz, est_viz, solution_eigen);
  
  // 创建带颜色的点云用于可视化
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_q_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  // 为不同点云着色以便区分
  colorize(src_viz, *src_colored, {195, 195, 195});      // 原始源点云：灰色
  colorize(tgt_viz, *tgt_colored, {89, 167, 230});       // 目标点云：蓝色
  colorize(est_viz, *est_q_colored, {238, 160, 61});     // 配准后的源点云：橙色

  // 创建PCL可视化窗口
  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  // 添加原始源点云到可视化窗口（灰色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
  // 添加目标点云到可视化窗口（蓝色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");
  // 添加配准后的源点云到可视化窗口（橙色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_q_colored, "est_q_blue");

  // 保持可视化窗口打开，直到用户关闭
  while (!viewer1.wasStopped()) {
    viewer1.spin();
  }
}
