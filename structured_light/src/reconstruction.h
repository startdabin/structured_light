#pragma once
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/distances.h>
#include<pcl/point_cloud.h>


class Reconstruction
{
public:
    Reconstruction()=default;
    ~Reconstruction()=default;

public:

    /**
    *   @brief  计算视差图
    * 
    *   @param  left_code_img   左相机编码图
    *   @param  right_code_img  右相机编码图
    * 
    *   @param  disparity_map  
    */
    cv::Mat calcDisparityMap(const cv::Mat& left_code_img, const cv::Mat& right_code_img) const;

    /**
    *   @brief  图像校正(极线)
    *
    *   @param  left_img            输入左相机图像
    *   @param  right_img           输入右相机图像
    *   @param  camera_param_path   相机参数路径
    *
    *   @param  rectified_img
    */
    cv::Mat rectifyImg(const cv::Mat& img, const cv::Mat& camera_matrix, const cv::Mat& distcoeffs, const cv::Mat& r, const cv::Mat& p) const;


    /**
    *   @brief  点云计算
    *
    *   @param  disparity           视差图
    *   @param  Q                   重投影矩阵
    *
    *   @param  point_cloud
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr calcPointCloud(const cv::Mat& disparity, const cv::Mat& Q) const;

};