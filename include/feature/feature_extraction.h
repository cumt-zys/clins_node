/*
 * CLINS: Continuous-Time Trajectory Estimation for LiDAR-Inertial System
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Kewei Hu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FEATURE_EXTRATION_H
#define FEATURE_EXTRATION_H

#include <clins/feature_cloud.h>
#include <feature/voxel_filter.h>
#include <ros/ros.h>
#include <sensor_data/lidar_data.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/tic_toc.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_data/calibration.hpp>

namespace clins {

// 定义平滑度结构体， 一个是值， 一个是索引
struct smoothness_t {
  float value;
  size_t ind;
};

// 通过平滑度值比较左右两个点的大小
struct by_value {
  bool operator()(smoothness_t const& left, smoothness_t const& right) {
    return left.value < right.value;
  }
};

// 定义特征提取类
class FeatureExtraction {
 public:
  // 有参构造， 传入配置节点
  FeatureExtraction(const YAML::Node& node);
  // 雷达数据处理， 传入雷达数据
  void LidarHandler(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
  // 函数重载， 传入原始点云  和  未失真点云
  void LidarHandler(RTPointCloud::Ptr raw_cloud, RTPointCloud::Ptr undistort_cloud);
  // 定位存储？
  void AllocateMemory();
  // 重置参数
  void ResetParameters();
  // 存储点云？
  bool CachePointCloud(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
  // 点云投影， 传入当前点云  深度图像  对应点云
  /// VLP_16 time = dsr * 2.304 * 1e-6 + firing * 55.296 * 1e-6;
  void ProjectOrganizedCloud(const VPointCloud::Ptr cur_cloud,
                             cv::Mat& dist_image,
                             RTPointCloud::Ptr corresponding_cloud);
  // 将点云投影到深度图
  /// project cloud to range mat
  void ProjectPointCloud(const RTPointCloud::Ptr cur_cloud, cv::Mat& dist_image,
                         RTPointCloud::Ptr corresponding_cloud);
  // 函数重载
  void ProjectPointCloud(const RTPointCloud::Ptr cur_cloud,
                         const RTPointCloud::Ptr undistort_cloud,
                         cv::Mat& dist_image,
                         RTPointCloud::Ptr cur_corresponding_cloud,
                         RTPointCloud::Ptr undistort_correspondig_cloud);
  // 点云提取
  void CloudExtraction();
  // 计算平滑度
  void CaculateSmoothness();
  // 标记遮挡点
  void MarkOccludedPoints();
  // 提取特征
  void ExtractFeatures();
  // 发布点云
  void PublishCloud(std::string frame_id);
  // 体素化点云过滤
  void RTPointCloudVoxelFilter(RTPointCloud::Ptr input_cloud,
                               RTPointCloud::Ptr output_cloud);
  // 点距离计算模板
  template <typename TPoint>
  float PointDistance(const TPoint& p) const;

  inline RTPointCloud::Ptr get_corner_features() const {
    return p_corner_cloud;
  }

  inline RTPointCloud::Ptr get_raw_corner_features() const {
    return p_raw_corner_cloud;
  }

  inline RTPointCloud::Ptr get_surface_features() const {
    return p_surface_cloud;
  }

  inline RTPointCloud::Ptr get_raw_surface_features() const {
    return p_raw_surface_cloud;
  }

  bool CheckMsgFields(const sensor_msgs::PointCloud2& cloud_msg,
                      std::string fields_name = "ring");

 private:
  ros::NodeHandle nh;

  ros::Subscriber sub_lidar;  // 订阅雷达点云
  ros::Publisher pub_corner_cloud;  // 发布角点
  ros::Publisher pub_surface_cloud;  // 发布平面点
  ros::Publisher pub_full_cloud;  // 发布所有点云
  ros::Publisher pub_feature_cloud;  // 发布特征点

  sensor_msgs::PointCloud2 cur_cloud_msg;  // 当前点云信息

  double min_distance_;  // 最小距离
  double max_distance_;  // 最大距离

  int n_scan;  // 激光线数
  int horizon_scan;  // 水平点数
  cv::Mat range_mat;  // 深度图
  RTPointCloud::Ptr p_full_cloud;  // 所有点云指针
  RTPointCloud::Ptr p_raw_cloud;  // 原始点云指针

  RTPointCloud::Ptr p_extracted_cloud;  // 提取点云指针

  std::string lidar_topic;  // 雷达话题

  std::vector<float> point_range_list;  // 点距离列表
  std::vector<int> point_column_id;  // 
  std::vector<int> start_ring_index;  // 开始ring通道下标
  std::vector<int> end_ring_index;  // 结束ring通道下标

  std::vector<smoothness_t> cloud_smoothness;  // 点云平滑度
  float* cloud_curvature;  // 点云曲率
  int* cloud_neighbor_picked;  // 点云被选邻近点
  int* cloud_label;  // 点云标签

  /// feature
  RTPointCloud::Ptr p_corner_cloud;
  RTPointCloud::Ptr p_surface_cloud;

  RTPointCloud::Ptr p_raw_corner_cloud;
  RTPointCloud::Ptr p_raw_surface_cloud;

  /// LOAM feature threshold
  float edge_threshold;
  float surf_threshold;

  VoxelFilter<RTPoint> down_size_filter;
  float odometry_surface_leaf_size;

  bool undistort_scan_before_extraction_;

  bool use_corner_feature_;
};

}  // namespace clins

#endif
