#pragma once


void remove_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers);
void remove_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers);
std::vector<float> find_pillar_planes_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold);
pcl::PointXYZRGB cv_to_pcl(cv::Point3i pt, std::uint32_t rgb);
pcl::PointCloud<pcl::PointXYZ>::Ptr pillar_corners_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
