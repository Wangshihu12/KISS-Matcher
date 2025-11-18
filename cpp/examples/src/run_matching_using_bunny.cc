// An example showing TEASER++ registration with FPFH features with the Stanford bunny model
#include <kiss_matcher/KISSMatcher.hpp>
#include <pcl/io/pcd_io.h>

#include "quatro/quatro_utils.h"

/**
 * [功能描述]：使用斯坦福兔子模型演示KISS-Matcher点云匹配和配准流程
 * 
 * 主要步骤：
 * 1. 加载源点云并应用SE(3)变换生成目标点云
 * 2. 使用KISS-Matcher进行特征匹配
 * 3. 分别使用Quatro和TEASER++算法进行点云配准
 * 4. 计算和比较两种方法的配准误差与耗时
 * 5. 可视化配准结果
 * 
 * @return 程序执行状态码
 */
int main() {
  // ============================================================
  // 步骤1: 加载源点云数据
  // ============================================================
  
  // 创建PLY文件读取器
  teaser::PLYReader reader;
  // 源点云容器
  teaser::PointCloud src_cloud;
  // 读取斯坦福兔子模型（.ply格式）
  auto status = reader.read("./data/bun_zipper.ply", src_cloud);
  // 获取点云中点的数量
  int N       = src_cloud.size();

  // ============================================================
  // 步骤2: 将点云数据转换为Eigen矩阵格式
  // ============================================================
  
  // 创建3xN的Eigen矩阵用于存储源点云坐标（每列代表一个点的xyz坐标）
  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, N);
  // 遍历所有点，将坐标从teaser格式转换为Eigen矩阵格式
  for (size_t i = 0; i < N; ++i) {
    // 将第i个点的xyz坐标填充到矩阵的第i列
    src.col(i) << src_cloud[i].x, src_cloud[i].y, src_cloud[i].z;
  }

  // ============================================================
  // 步骤3: 转换为齐次坐标并应用SE(3)变换
  // ============================================================
  
  // 创建4xN的齐次坐标矩阵（最后一行为1，用于支持平移变换）
  Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
  src_h.resize(4, src.cols());
  // 前3行：xyz坐标
  src_h.topRows(3)    = src;
  // 最后一行：全1（齐次坐标的标识）
  src_h.bottomRows(1) = Eigen::Matrix<double, 1, Eigen::Dynamic>::Ones(N);

  // 构造一个SE(3)变换矩阵T（4x4），用于生成目标点云
  // SE(3)表示3D空间中的刚体变换（旋转+平移）
  Eigen::Matrix4d T          = Eigen::Matrix4d::Identity();  // 初始化为单位矩阵
  // 生成绕x轴旋转60度的旋转矩阵（3x3）
  Eigen::Matrix3d random_rot = get3DRot(60, 0, 0);
  // 将旋转矩阵填充到变换矩阵的左上角3x3子块
  T.block<3, 3>(0, 0)        = random_rot;
  // 设置x方向的平移量
  T(0, 3)                    = -0.02576939;
  // 设置y方向的平移量
  T(1, 3)                    = -0.037705398;

  // 应用变换矩阵T到源点云，得到目标点云的齐次坐标（4xN）
  Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T * src_h;
  // 提取前3行得到目标点云的xyz坐标（3xN）
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt   = tgt_h.topRows(3);

  // ============================================================
  // 步骤4: 将目标点云转换回teaser格式（用于后续处理）
  // ============================================================
  
  // 创建teaser格式的目标点云容器
  teaser::PointCloud tgt_cloud;
  // 遍历所有点，将Eigen矩阵格式转换为teaser点云格式
  for (size_t i = 0; i < tgt.cols(); ++i) {
    tgt_cloud.push_back({static_cast<float>(tgt(0, i)),  // x坐标
                         static_cast<float>(tgt(1, i)),  // y坐标
                         static_cast<float>(tgt(2, i))}); // z坐标
  }

  // ============================================================
  // 步骤5: 配置KISS-Matcher并进行点云匹配
  // ============================================================
  
  // 设置鲁棒估计器的模式（最大核方法）
  std::string robin_mode                 = "max_core";
  // 设置体素下采样的大小（单位：米）
  float voxel_size                       = 0.01;
  // 创建KISS-Matcher配置对象
  kiss_matcher::KISSMatcherConfig config = kiss_matcher::KISSMatcherConfig(voxel_size);
  // 创建KISS-Matcher匹配器实例
  kiss_matcher::KISSMatcher matcher(config);

  // 执行点云匹配，返回匹配的源点云和目标点云对应点
  const auto &[src_matched, tgt_matched] = matcher.match(src, tgt);

  // ============================================================
  // 步骤6: 将匹配结果转换为Eigen矩阵格式
  // ============================================================
  
  // 获取匹配点对的数量
  int M = src_matched.size();
  // 创建3xM矩阵存储匹配的源点云（M为匹配点数）
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched_eigen(3, M);
  // 创建3xM矩阵存储匹配的目标点云
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched_eigen(3, M);

  // 遍历所有匹配点对，填充到Eigen矩阵中
  for (int m = 0; m < M; ++m) {
    // 将匹配的源点转换为double类型并填充到矩阵第m列
    src_matched_eigen.col(m) << src_matched[m].cast<double>();
    // 将匹配的目标点转换为double类型并填充到矩阵第m列
    tgt_matched_eigen.col(m) << tgt_matched[m].cast<double>();
  }

  // ============================================================
  // 步骤7: 使用Quatro算法进行鲁棒配准
  // ============================================================
  
  // 声明Quatro和TEASER++的参数对象
  teaser::RobustRegistrationSolver::Params quatro_param, teaser_param;
  // 获取Quatro算法的参数（噪声界限的一半、算法名称、鲁棒模式）
  getParams(NOISE_BOUND / 2, "Quatro", robin_mode, quatro_param);
  // 记录Quatro算法开始时间
  std::chrono::steady_clock::time_point begin_q = std::chrono::steady_clock::now();
  // 创建Quatro求解器实例
  teaser::RobustRegistrationSolver Quatro(quatro_param);
  // 求解配准问题（根据匹配点对估计变换矩阵）
  Quatro.solve(src_matched_eigen, tgt_matched_eigen);
  // 记录Quatro算法结束时间
  std::chrono::steady_clock::time_point end_q = std::chrono::steady_clock::now();
  // 获取Quatro算法的配准结果（旋转矩阵和平移向量）
  auto solution_by_quatro                     = Quatro.getSolution();

  // ============================================================
  // 步骤8: 使用TEASER++算法进行鲁棒配准
  // ============================================================
  
  // 获取TEASER++算法的参数
  getParams(NOISE_BOUND / 2, "TEASER", robin_mode, teaser_param);
  // 记录TEASER++算法开始时间
  std::chrono::steady_clock::time_point begin_t = std::chrono::steady_clock::now();
  // 创建TEASER++求解器实例
  teaser::RobustRegistrationSolver TEASER(teaser_param);
  // 求解配准问题
  TEASER.solve(src_matched_eigen, tgt_matched_eigen);
  // 记录TEASER++算法结束时间
  std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
  // 获取TEASER++算法的配准结果
  auto solution_by_teaser                     = TEASER.getSolution();

  // ============================================================
  // 步骤9: 输出Quatro算法的配准结果
  // ============================================================
  
  std::cout << "=====================================" << std::endl;
  std::cout << "           Quatro Results            " << std::endl;
  std::cout << "=====================================" << std::endl;
  // 声明旋转误差和平移误差变量
  double rot_error_quatro, ts_error_quatro;
  // 计算Quatro估计结果与真实变换T之间的误差
  calcErrors(T,
             solution_by_quatro.rotation,     // Quatro估计的旋转矩阵
             solution_by_quatro.translation,  // Quatro估计的平移向量
             rot_error_quatro,                // 输出：旋转误差（度）
             ts_error_quatro);                // 输出：平移误差（米）
  // 输出旋转误差（单位：度）
  std::cout << "Error (deg): " << rot_error_quatro << std::endl;
  // 输出平移误差（单位：米）
  std::cout << "Estimated translation (m): " << ts_error_quatro << std::endl;
  // 输出Quatro算法的执行时间（单位：秒）
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_q - begin_q).count() /
                   1000000.0
            << std::endl;

  // ============================================================
  // 步骤10: 输出TEASER++算法的配准结果
  // ============================================================
  
  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "=====================================" << std::endl;
  // 声明TEASER++的误差变量
  double rot_error_teaser, ts_error_teaser;
  // 计算TEASER++估计结果与真实变换T之间的误差
  calcErrors(T,
             solution_by_teaser.rotation,     // TEASER++估计的旋转矩阵
             solution_by_teaser.translation,  // TEASER++估计的平移向量
             rot_error_teaser,                // 输出：旋转误差（度）
             ts_error_teaser);                // 输出：平移误差（米）
  // 输出TEASER++的结果
  std::cout << "Error (deg): " << rot_error_teaser << std::endl;
  std::cout << "Estimated translation (m): " << ts_error_teaser << std::endl;
  // 输出TEASER++算法的执行时间（单位：秒）
  std::cout << "Time taken (s): "
            << std::chrono::duration_cast<std::chrono::microseconds>(end_t - begin_t).count() /
                   1000000.0
            << std::endl;

  // ============================================================
  // 步骤11: 准备可视化数据
  // ============================================================
  
  // 创建PCL格式的点云对象用于可视化
  pcl::PointCloud<pcl::PointXYZ> src_raw;   // 源点云（原始）
  pcl::PointCloud<pcl::PointXYZ> tgt_raw;   // 目标点云（原始）
  pcl::PointCloud<pcl::PointXYZ> est_q, est_t;  // Quatro和TEASER++的配准结果

  // 将teaser格式的点云转换为PCL格式
  for (int k = 0; k < src_cloud.size(); ++k) {
    // 添加源点云的点
    src_raw.push_back(pcl::PointXYZ(src_cloud[k].x, src_cloud[k].y, src_cloud[k].z));
    // 添加目标点云的点
    tgt_raw.push_back(pcl::PointXYZ(tgt_cloud[k].x, tgt_cloud[k].y, tgt_cloud[k].z));
  }
  
  // 构造Quatro的变换矩阵（4x4）并应用到源点云
  Eigen::Matrix4f solution_eigen      = Eigen::Matrix4f::Identity();
  // 填充旋转矩阵部分（左上角3x3）
  solution_eigen.block<3, 3>(0, 0)    = solution_by_quatro.rotation.cast<float>();
  // 填充平移向量部分（右上角3x1）
  solution_eigen.topRightCorner(3, 1) = solution_by_quatro.translation.cast<float>();
  // 输出Quatro的变换矩阵
  std::cout << solution_eigen << std::endl;
  // 使用Quatro估计的变换矩阵变换源点云，得到配准后的点云
  pcl::transformPointCloud(src_raw, est_q, solution_eigen);

  // 构造TEASER++的变换矩阵并应用到源点云
  solution_eigen.block<3, 3>(0, 0)    = solution_by_teaser.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution_by_teaser.translation.cast<float>();
  std::cout << solution_eigen << std::endl;
  // 使用TEASER++估计的变换矩阵变换源点云
  pcl::transformPointCloud(src_raw, est_t, solution_eigen);

  // ============================================================
  // 步骤12: 为点云着色准备可视化
  // ============================================================
  
  // 创建带颜色信息的点云对象（智能指针）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);     // 源点云（红色）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);     // 目标点云（绿色）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_q_colored(new pcl::PointCloud<pcl::PointXYZRGB>);   // Quatro结果（蓝色）
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr est_t_colored(new pcl::PointCloud<pcl::PointXYZRGB>);   // TEASER++结果（品红色）

  // 如果需要保存配准后的点云，可以使用下面这行代码
  // pcl::io::savePCDFileASCII("your_path/src_warped.pcd", est_t);

  // 为各个点云着色（RGB格式）
  colorize(src_raw, *src_colored, {255, 0, 0});      // 源点云着红色
  colorize(tgt_raw, *tgt_colored, {0, 255, 0});      // 目标点云着绿色
  colorize(est_q, *est_q_colored, {0, 0, 255});      // Quatro结果着蓝色
  colorize(est_t, *est_t_colored, {255, 0, 255});    // TEASER++结果着品红色

  // ============================================================
  // 步骤13: 创建可视化窗口并显示所有点云
  // ============================================================
  
  // 创建PCL可视化器对象
  pcl::visualization::PCLVisualizer viewer1("Simple Cloud Viewer");
  // 添加源点云到可视化器（红色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(src_colored, "src_red");
  // 添加目标点云到可视化器（绿色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(tgt_colored, "tgt_green");
  // 添加Quatro配准结果到可视化器（蓝色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_q_colored, "est_q_blue");
  // 添加TEASER++配准结果到可视化器（品红色）
  viewer1.addPointCloud<pcl::PointXYZRGB>(est_t_colored, "est_t_magenta");

  // 保持可视化窗口打开，直到用户关闭
  while (!viewer1.wasStopped()) {
    viewer1.spin();  // 更新可视化窗口
  }
}
