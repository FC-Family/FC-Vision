#ifndef PATH_SEARCHING_GEOMETRY_UTILS_HPP
#define PATH_SEARCHING_GEOMETRY_UTILS_HPP

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace visibility_utils
{

inline void meshToPointCloud(
    const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXi& faces,
    double resolution,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  cloud->clear();
  if (faces.rows() == 0 || resolution <= 0.0) return;

  std::vector<double> areas(faces.rows());
  double total_area = 0.0;
  for (int i = 0; i < faces.rows(); ++i)
  {
    const Eigen::Vector3d v1 = vertices.row(faces(i, 0));
    const Eigen::Vector3d v2 = vertices.row(faces(i, 1));
    const Eigen::Vector3d v3 = vertices.row(faces(i, 2));
    areas[i] = 0.5 * ((v2 - v1).cross(v3 - v1)).norm();
    total_area += areas[i];
  }
  if (total_area <= 0.0) return;

  const int total_points = std::max(1, static_cast<int>(total_area / (resolution * resolution)));
  std::vector<double> cumulative_areas(areas.size());
  std::partial_sum(areas.begin(), areas.end(), cumulative_areas.begin());

  cloud->points.resize(total_points);
  std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  for (int i = 0; i < total_points; ++i)
  {
    const double random_area = dist(rng) * total_area;
    auto it = std::lower_bound(cumulative_areas.begin(), cumulative_areas.end(), random_area);
    const int triangle_idx = std::min<int>(
        std::distance(cumulative_areas.begin(), it), faces.rows() - 1);

    const Eigen::Vector3d v1 = vertices.row(faces(triangle_idx, 0));
    const Eigen::Vector3d v2 = vertices.row(faces(triangle_idx, 1));
    const Eigen::Vector3d v3 = vertices.row(faces(triangle_idx, 2));
    const double sqrt_r1 = std::sqrt(dist(rng));
    const double u = 1.0 - sqrt_r1;
    const double v = dist(rng) * sqrt_r1;
    const Eigen::Vector3d point = u * v1 + v * v2 + (1.0 - u - v) * v3;
    cloud->points[i] = pcl::PointXYZ(point.x(), point.y(), point.z());
  }
}

inline void pointCloudToEigen(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    Eigen::Matrix4Xd& matrix)
{
  matrix = Eigen::Matrix4Xd::Zero(4, cloud->points.size());
  for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
  {
    matrix(0, i) = cloud->points[i].x;
    matrix(1, i) = cloud->points[i].y;
    matrix(2, i) = cloud->points[i].z;
    matrix(3, i) = 1.0;
  }
}

inline std::vector<Eigen::Vector3d> sampleLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double endpoint_inflation,
    double resolution)
{
  std::vector<Eigen::Vector3d> samples;
  Eigen::Vector3d direction = end - start;
  const double length = direction.norm();
  const double sampled_length = length - 2.0 * endpoint_inflation;
  if (length <= endpoint_inflation || sampled_length <= 0.0 || resolution <= 0.0) return samples;

  direction.normalize();
  const Eigen::Vector3d sampled_start = start + endpoint_inflation * direction;
  const int sample_count = static_cast<int>(sampled_length / resolution);
  samples.reserve(sample_count);
  for (int i = 1; i <= sample_count; ++i)
  {
    const double distance = resolution * i;
    if (distance > sampled_length) break;
    samples.push_back(sampled_start + direction * distance);
  }
  return samples;
}

}  // namespace visibility_utils

#endif
