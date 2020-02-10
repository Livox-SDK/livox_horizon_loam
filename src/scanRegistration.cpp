// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               Livox@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <string>
#include <vector>

#include "loam_horizon/common.h"
#include "loam_horizon/tic_toc.h"

using std::atan2;
using std::cos;
using std::sin;

constexpr bool dbg_show_id = false;
constexpr bool b_only_1_scan = false;
constexpr bool b_viz_curv = false;

constexpr bool b_normalize_curv = true;

const double scanPeriod = 0.1;

const int systemDelay = 0;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

ros::Publisher pub_curvature;

bool PUB_EACH_LINE = true;

double MINIMUM_RANGE = 0.1;
double THRESHOLD_FLAT = 0.01;
double THRESHOLD_SHARP = 0.01;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        thres * thres)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

template <typename PointT>
void VisualizeCurvature(float *v_curv, int *v_label,
                        const pcl::PointCloud<PointT> &pcl_in,
                        const std_msgs::Header &hdr) {
  ROS_ASSERT(pcl_in.size() < 400000);

  /// Same marker attributes
  visualization_msgs::Marker txt_mk;
  txt_mk.header = hdr;
  txt_mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  txt_mk.ns = "default";
  txt_mk.id = 0;
  txt_mk.action = visualization_msgs::Marker::ADD;
  txt_mk.pose.orientation.x = 0;
  txt_mk.pose.orientation.y = 0;
  txt_mk.pose.orientation.z = 0;
  txt_mk.pose.orientation.w = 1;
  txt_mk.scale.z = 0.05;
  txt_mk.color.a = 1;
  txt_mk.color.r = 0;
  txt_mk.color.g = 1;
  txt_mk.color.b = 0;

  static visualization_msgs::MarkerArray curv_txt_msg;
  // for (size_t i = 0; i < curv_txt_msg.markers.size(); ++i) {
  //   auto &mk_i = curv_txt_msg.markers[i];
  //   mk_i = txt_mk;
  //   mk_i.header.stamp = txt_mk.header.stamp - ros::Duration(0.001);
  //   mk_i.action = visualization_msgs::Marker::DELETE;
  //   mk_i.text = "";
  //   mk_i.ns = "old";
  //   mk_i.color.a = 0;
  // }
  // pub_curvature.publish(curv_txt_msg);
  // ros::Rate r(200);
  // r.sleep();

  /// Marger array message
  static size_t pre_pt_num = 0;
  size_t pt_num = pcl_in.size();

  if (pre_pt_num == 0) {
    curv_txt_msg.markers.reserve(400000);
  }
  if (pre_pt_num > pt_num) {
    curv_txt_msg.markers.resize(pre_pt_num);
  } else {
    curv_txt_msg.markers.resize(pt_num);
  }

  int edge_num = 0, edgeless_num = 0, flat_num = 0, flatless_num = 0, nn = 0;

  /// Add marker and namespace
  for (size_t i = 0; i < pcl_in.size(); ++i) {
    auto curv = v_curv[i];
    auto label = v_label[i];  /// -1: flat, 0: less-flat, 1:less-edge, 2:edge
    const auto &pt = pcl_in[i];

    switch (label) {
      case 2: {
        /// edge
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "edge";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 1;
        mk_i.color.r = 1;
        mk_i.color.g = 0;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        edge_num++;
        break;
      }
      case 1: {
        /// less edge
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "edgeless";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0.5;
        mk_i.color.r = 0.5;
        mk_i.color.g = 0;
        mk_i.color.b = 0.8;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        edgeless_num++;
        break;
      }
      case 0: {
        /// less flat
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "flatless";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0.5;
        mk_i.color.r = 0;
        mk_i.color.g = 0.5;
        mk_i.color.b = 0.8;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        flatless_num++;
        break;
      }
      case -1: {
        /// flat
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "flat";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 1;
        mk_i.color.r = 0;
        mk_i.color.g = 1;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);
        /// debug
        if (dbg_show_id) {
          mk_i.text = std::to_string(i);
        }

        flat_num++;
        break;
      }
      default: {
        /// Un-reliable
        /// Do nothing for label=99
        // ROS_ASSERT_MSG(false, "%d", label);
        auto &mk_i = curv_txt_msg.markers[i];
        mk_i = txt_mk;
        mk_i.ns = "unreliable";
        mk_i.id = i;
        mk_i.pose.position.x = pt.x;
        mk_i.pose.position.y = pt.y;
        mk_i.pose.position.z = pt.z;
        mk_i.color.a = 0;
        mk_i.color.r = 0;
        mk_i.color.g = 0;
        mk_i.color.b = 0;
        char cstr[10];
        snprintf(cstr, 9, "%.2f", curv);
        mk_i.text = std::string(cstr);

        nn++;
        break;
      }
    }
  }
  ROS_INFO("edge/edgeless/flatless/flat/nn num: [%d / %d / %d / %d / %d] - %lu",
           edge_num, edgeless_num, flatless_num, flat_num, nn, pt_num);

  /// Delete old points
  if (pre_pt_num > pt_num) {
    ROS_WARN("%lu > %lu", pre_pt_num, pt_num);
    // curv_txt_msg.markers.resize(pre_pt_num);
    for (size_t i = pt_num; i < pre_pt_num; ++i) {
      auto &mk_i = curv_txt_msg.markers[i];
      mk_i.action = visualization_msgs::Marker::DELETE;
      mk_i.color.a = 0;
      mk_i.color.r = 0;
      mk_i.color.g = 0;
      mk_i.color.b = 0;
      mk_i.ns = "old";
      mk_i.text = "";
    }
  }
  pre_pt_num = pt_num;

  pub_curvature.publish(curv_txt_msg);
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    } else
      return;
  }

  TicToc t_whole;
  TicToc t_prepare;
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  pcl::PointCloud<PointType> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;

  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

  int cloudSize = laserCloudIn.points.size();
  int count = cloudSize;
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;
    point.intensity = laserCloudIn.points[i].intensity;
    point.curvature = laserCloudIn.points[i].curvature;
    int scanID = 0;
    if (N_SCANS == 6) {
      scanID = (int)point.intensity;
    }
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;
  printf("points size %d \n", cloudSize);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < N_SCANS; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
    // ROS_INFO("scan %d start-end [%d, %d]", i, scanStartInd[i],
    // scanEndInd[i]);
  }

  printf("prepare time %f \n", t_prepare.toc());

  int kNumCurvSize = 5;
  constexpr int kNumRegion = 50;       // 6
  constexpr int kNumEdge = 2;          // 2
  constexpr int kNumFlat = 4;          // 4
  constexpr int kNumEdgeNeighbor = 5;  // 5;
  constexpr int kNumFlatNeighbor = 5;  // 5;
  float kThresholdSharp = 50;          // 0.1;
  float kThresholdFlat = 30;           // 0.1;
  constexpr float kThresholdLessflat = 0.1;

  constexpr float kDistanceFaraway = 25;
  for (int i = 5; i < cloudSize - 5; i++) {
    float dis = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
    if (dis > kDistanceFaraway) {
      kNumCurvSize = 2;
    }
    float diffX = 0, diffY = 0, diffZ = 0;
    for (int j = 1; j <= kNumCurvSize; ++j) {
      diffX += laserCloud->points[i - j].x + laserCloud->points[i + j].x;
      diffY += laserCloud->points[i - j].y + laserCloud->points[i + j].y;
      diffZ += laserCloud->points[i - j].z + laserCloud->points[i + j].z;
    }
    diffX -= 2 * kNumCurvSize * laserCloud->points[i].x;
    diffY -= 2 * kNumCurvSize * laserCloud->points[i].y;
    diffZ -= 2 * kNumCurvSize * laserCloud->points[i].z;
    /*
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x +
                  laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                  laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                  laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                  laserCloud->points[i + 3].x + laserCloud->points[i + 4].x +
                  laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y +
                  laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                  laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                  laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                  laserCloud->points[i + 3].y + laserCloud->points[i + 4].y +
                  laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z +
                  laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                  laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                  laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                  laserCloud->points[i + 3].z + laserCloud->points[i + 4].z +
                  laserCloud->points[i + 5].z;
                  */

    float tmp2 = diffX * diffX + diffY * diffY + diffZ * diffZ;
    float tmp = sqrt(tmp2);

    cloudCurvature[i] = tmp2;
    if (b_normalize_curv) {
      /// use normalized curvature
      cloudCurvature[i] = tmp / (2 * kNumCurvSize * dis + 1e-3);
    }
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    /// Mark un-reliable points
    constexpr float kMaxFeatureDis = 1e4;
    if (fabs(dis) > kMaxFeatureDis || fabs(dis) < 1e-4 || !std::isfinite(dis)) {
      cloudLabel[i] = 99;
      cloudNeighborPicked[i] = 1;
    }
  }

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
    float dis = laserCloud->points[i].x * laserCloud->points[i].x +
                laserCloud->points[i].y * laserCloud->points[i].y +
                laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.00015 * dis && diff2 > 0.00015 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  TicToc t_pts;

  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  if (b_normalize_curv) {
    kThresholdFlat = THRESHOLD_FLAT;
    kThresholdSharp = THRESHOLD_SHARP;
  }

  float t_q_sort = 0;
  for (int i = 0; i < N_SCANS; i++) {
    if (scanEndInd[i] - scanStartInd[i] < kNumCurvSize) continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(
        new pcl::PointCloud<PointType>);

    /// debug
    if (b_only_1_scan && i > 0) {
      break;
    }

    for (int j = 0; j < kNumRegion; j++) {
      int sp =
          scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / kNumRegion;
      int ep = scanStartInd[i] +
               (scanEndInd[i] - scanStartInd[i]) * (j + 1) / kNumRegion - 1;
      //      ROS_INFO("scan [%d], id from-to [%d-%d] in [%d-%d]", i, sp, ep,
      //               scanStartInd[i], scanEndInd[i]);

      TicToc t_tmp;
      //      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
      // sort the curvatures from small to large
      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] <
              cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      float SumCurRegion = 0.0;
      float MaxCurRegion = cloudCurvature[cloudSortInd[ep]];  //the largest curvature in sp ~ ep
      for (int k = ep - 1; k >= sp; k--) {
        SumCurRegion += cloudCurvature[cloudSortInd[k]];
      }

      if (MaxCurRegion > 3 * SumCurRegion)
        cloudNeighborPicked[cloudSortInd[ep]] = 1;

      t_q_sort += t_tmp.toc();

      if (true) {
        for (int tt = sp; tt < ep - 1; ++tt) {
          ROS_ASSERT(cloudCurvature[cloudSortInd[tt]] <=
                     cloudCurvature[cloudSortInd[tt + 1]]);
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] != 0) continue;

        if (cloudCurvature[ind] > kThresholdSharp) {
          largestPickedNum++;
          if (largestPickedNum <= kNumEdge) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
            // ROS_INFO("pick sharp at sort_id [%d], primary id[%d]", k, ind);
            // if (ind == 211 || ind == 212 || ind == 213 || ind == 214) {
            //   const auto &pt = laserCloud->points[ind];
            //   printf("%d-[%f, %f, %f]\n", ind, pt.x, pt.y, pt.z);
            // }
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= kNumEdgeNeighbor; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -kNumEdgeNeighbor; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] != 0) continue;

        if (cloudCurvature[ind] < kThresholdFlat) {
          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(laserCloud->points[ind]);
          cloudNeighborPicked[ind] = 1;

          smallestPickedNum++;
          if (smallestPickedNum >= kNumFlat) {
            break;
          }

          for (int l = 1; l <= kNumFlatNeighbor; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -kNumFlatNeighbor; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0 && cloudCurvature[k] < kThresholdLessflat) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    surfPointsLessFlat += surfPointsFlat;
    cornerPointsLessSharp += cornerPointsSharp;
    /// Whether downsample less-flat points
    if (false) {
      pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
      pcl::VoxelGrid<PointType> downSizeFilter;
      downSizeFilter.setInputCloud(surfPointsLessFlatScan);
      downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
      downSizeFilter.filter(surfPointsLessFlatScanDS);
      surfPointsLessFlat += surfPointsLessFlatScanDS;
    } else {
      surfPointsLessFlat += *surfPointsLessFlatScan;
    }
  }
  printf("sort q time %f \n", t_q_sort);
  printf("seperate points time %f \n", t_pts.toc());

  if (false) {
    removeClosedPointCloud(*laserCloud, *laserCloud, MINIMUM_RANGE);
    removeClosedPointCloud(cornerPointsLessSharp, cornerPointsLessSharp,
                           MINIMUM_RANGE);
    removeClosedPointCloud(cornerPointsSharp, cornerPointsSharp, MINIMUM_RANGE);
    removeClosedPointCloud(surfPointsFlat, surfPointsFlat, MINIMUM_RANGE);
    removeClosedPointCloud(surfPointsLessFlat, surfPointsLessFlat,
                           MINIMUM_RANGE);
  }

  /// Visualize curvature
  if (b_viz_curv) {
    std_msgs::Header ros_hdr = laserCloudMsg->header;
    ros_hdr.frame_id = "/aft_mapped";
    VisualizeCurvature(cloudCurvature, cloudLabel, *laserCloud, ros_hdr);
  }

  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/aft_mapped";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/aft_mapped";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/aft_mapped";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/aft_mapped";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/aft_mapped";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  // pub each scam
  if (PUB_EACH_LINE) {
    for (int i = 0; i < N_SCANS; i++) {
      sensor_msgs::PointCloud2 scanMsg;
      pcl::toROSMsg(laserCloudScans[i], scanMsg);
      scanMsg.header.stamp = laserCloudMsg->header.stamp;
      scanMsg.header.frame_id = "/aft_mapped";
      pubEachScan[i].publish(scanMsg);
    }
  }

  printf("scan registration time %f ms *************\n", t_whole.toc());
  if (t_whole.toc() > 100) ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  nh.param<int>("scan_line", N_SCANS, 6); // Horizon has 6 scan lines
  nh.param<double>("threshold_flat", THRESHOLD_FLAT, 0.01);
  nh.param<double>("threshold_sharp", THRESHOLD_SHARP, 0.01);

  printf("scan line number %d \n", N_SCANS);

  if (N_SCANS != 6) {
    printf("only support livox horizon lidar!");
    return 0;
  }

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/livox_undistort", 100, laserCloudHandler);

  pubLaserCloud =
      nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

  pubCornerPointsSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

  pubCornerPointsLessSharp =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

  pubSurfPointsFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

  pubSurfPointsLessFlat =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

  pubRemovePoints =
      nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

  pub_curvature =
      nh.advertise<visualization_msgs::MarkerArray>("/curvature", 100);

  if (PUB_EACH_LINE) {
    for (int i = 0; i < N_SCANS; i++) {
      ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>(
          "/laser_scanid_" + std::to_string(i), 100);
      pubEachScan.push_back(tmp);
    }
  }
  ros::spin();

  return 0;
}
