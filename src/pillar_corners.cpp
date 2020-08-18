#include <Eigen/Geometry>
#include "pcl/point_cloud.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>
#include <chrono>

#include "pillar_corners.hpp"

//using namespace std::literals::chrono_literals;

//using namespace std::chrono_literals;


//K: [672.8525634486431, 0.0, 399.48117481367757, 0.0, 672.758607019733, 300.6428102963154, 0.0, 0.0, 1.0]
double fx =672.8525634486431;
double fy = 672.758607019733;
double cx = 399.48117481367757;
double cy = 300.6428102963154;

int HEIGHT = 592;
int WIDTH = 800;

struct My_line{
    Eigen::Vector3f line;
    Eigen::Vector3f point;
};

using namespace cv;
using namespace std;
//using namespace std::chrono_literals;

float rgb2float(uint8_t r, uint8_t g, uint8_t b){
    uint32_t color = r << 16 | g << 8 | b;
    return *reinterpret_cast<float*>(&color);
}




pcl::PointXYZRGB addcol(pcl::PointXYZ p, uint8_t r = 255, uint8_t g = 255, uint8_t b = 255){
    pcl::PointXYZRGB rgb;
    rgb.rgb = rgb2float(r,g,b);
    rgb.x = p.x;
    rgb.y = p.y;
    rgb.z = p.z;
    return rgb;
}


pcl::PointXYZRGB cv_to_pcl(Point3i pt, uint32_t rgb){
    pcl::PointXYZRGB p;
    p.x = ((double)pt.x - cx)/fx * (double)pt.z / 1000;
    p.y = ((double)pt.y - cy)/fy * (double)pt.z / 1000;
    p.z = (double)pt.z / 1000;
    p.rgb = *reinterpret_cast<float*>(&rgb);
    return p;
}

vector<float> find_pillar_planes_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold){
//finds 4 planes which represent pillar. First find two parallel planes, then 2 perpendicular
//TODO try finding perpendicular planes by projection -> more robust ? 
  vector<float> plane_coeff;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outliers (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}
  
  //for(int i = 0; i < inliers->indices.size(); i++){
      //cloud->points[inliers->indices[i]].rgb = rgb2float(255,0,0);
  //}
  //pcl::PointXYZ pt;
  //pcl::PointXYZRGB rgb_pt;

  remove_cloud(cloud, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}


  remove_cloud(outliers, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}


  remove_cloud(outliers, inliers, outliers);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f(coefficients->values[0],coefficients->values[1],coefficients->values[2]));
  seg.setInputCloud(outliers);
  seg.segment(*inliers,*coefficients);
  for(int i = 0; i < 4; i++){plane_coeff.push_back(coefficients->values[i]);}

  return plane_coeff;
}

void remove_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZ>::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*outliers);
      outliers->height = outliers->points.size();
      outliers->width = 1;
}

void remove_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointIndices::Ptr indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers){
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*outliers);
      outliers->height = outliers->points.size();
      outliers->width = 1;
}

pcl::ModelCoefficients::Ptr find_plane_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr outliers = nullptr){

  //pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if(outliers != nullptr){
      remove_cloud(cloud, inliers,outliers);
  }
  return coefficients;
}

void convert_to_cloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int limit = 100000){
    cv::Mat image;
    cv::FileStorage fs;
    fs.open(name, cv::FileStorage::READ);
    fs["matName"] >> image;
    imshow("name",image);
    //waitKey();
    pcl::PointXYZRGB pt;
    Point3i tmp;
    uint8_t r = 255,b=255,g=255;
    uint32_t rgb =(static_cast<std::uint32_t>(r) << 16 |
              static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));;
    for(int i = 0; i < HEIGHT; i++){
        for(int j = 0; j < WIDTH; j++){
            tmp.x = j;
            tmp.y = i;
            tmp.z = image.at<uint16_t>(i,j);
            if(tmp.z != 65535 && tmp.z < limit){
                pt = cv_to_pcl(tmp, rgb);
                cloud->points.push_back(pt);
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_plane_cloud(pcl::ModelCoefficients::Ptr coefficients){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB p;
    for(float i = -20; i < 20; i +=0.5){
        for(float j = -20; j < 120; j += 0.5){
            //p.x = i; p.y = j;
            //p.z = (-coefficients->values[3] - coefficients->values[0] * i - coefficients->values[1] * j)/coefficients->values[2];
            p.x = i; p.z = j;
            p.y = (-coefficients->values[3] - coefficients->values[0] * i - coefficients->values[2] * j)/coefficients->values[1];
            p.rgb = rgb2float(0, 255,0);
            cloud->push_back(p);
        }
    }
    return cloud;
}

//pcl::PointXYZRGB Eigen_to_pcl(Eigen::Vector3f a, uint8_t r = 255, uint8_t g = 255, uint8_t bl = 255){
    //pcl::PointXYZRGB b;
    //b.x = a.x(); b.y = a.y(); b.z = a.z(); b.rgb = rgb2float(r,g,bl);
    //return b;
//}

pcl::PointXYZ Eigen_to_pcl(Eigen::Vector3f a){
    pcl::PointXYZ b;
    b.x = a.x(); b.y = a.y(); b.z = a.z();
    return b;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_pillar_base_corners(pcl::ModelCoefficients::Ptr plane, vector<My_line> lines){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    double t;
    Eigen::Vector3f norm(plane->values[0],plane->values[1],plane->values[2]);
    double d = plane->values[3];
    Eigen::Vector3f intersection;
    pcl::PointXYZ pt;

    for(int i =0; i < 4; i++){
        t = -(d+ lines[i].point.dot(norm))/(lines[i].line.dot(norm));
        intersection = lines[i].point + t*lines[i].line;
        cloud->push_back(Eigen_to_pcl(intersection));
    }
    return cloud;
}

            
My_line plane_intersection(Eigen::Vector4f p1, Eigen::Vector4f p2){
    Eigen::Vector3f v1(p1[0],p1[1],p1[2]);
    Eigen::Vector3f v2(p2[0],p2[1],p2[2]);

    Eigen::Vector3f lin_v = v1.cross(v2);
    Eigen::Vector3f lin_p;
    lin_p[0] = 0;
    lin_p[2] = ((p2[1]/p1[1])*p1[3] - p2[3])/(p2[2] - p1[2]*p2[1]/p1[1]);
    lin_p[1] = (-p1[2]*lin_p[2] - p1[3])/ p1[1];
    My_line my_line;
    my_line.line = lin_v;
    my_line.point = lin_p;
    return my_line;
}
    
    //define point clouds

pcl::PointCloud<pcl::PointXYZ>::Ptr pillar_corners_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pillar_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pillar_corners ;

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(0.1f,0.1f,0.1f);
    grid.filter(*voxel_cloud);
    //find plane by RANSAC
    coefficients = find_plane_ransac(voxel_cloud,0.3, pillar_cloud);
    //find_plane_ransac(cloud,0.3, pillar_cloud);

    vector<float> planes = find_pillar_planes_ransac(pillar_cloud, 0.15);
    Eigen::Vector4f p1,p2;
    vector<My_line> lines;

    for(int n = 0; n < 4; n++){
    for(int i = 0; i < 4 ; i++){
        if(n < 2){
            p1[i] = planes[i+n*4];
            p2[i] = planes[i+n*4+8];
        }
        else{
            p1[i] = planes[i + n*4 - 8];
            p2[i] = planes[i + 20 - n*4];
        }
    }
    My_line my_line = plane_intersection(p1,p2);
    lines.push_back(my_line);
    }
    pillar_corners = get_pillar_base_corners(coefficients,lines);
    return pillar_corners;
}

        

