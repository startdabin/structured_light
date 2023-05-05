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
    *   @brief  �����Ӳ�ͼ
    * 
    *   @param  left_code_img   ���������ͼ
    *   @param  right_code_img  ���������ͼ
    * 
    *   @param  disparity_map  
    */
    cv::Mat calcDisparityMap(const cv::Mat& left_code_img, const cv::Mat& right_code_img) const;

    /**
    *   @brief  ͼ��У��(����)
    *
    *   @param  left_img            ���������ͼ��
    *   @param  right_img           ���������ͼ��
    *   @param  camera_param_path   �������·��
    *
    *   @param  rectified_img
    */
    cv::Mat rectifyImg(const cv::Mat& img, const cv::Mat& camera_matrix, const cv::Mat& distcoeffs, const cv::Mat& r, const cv::Mat& p) const;


    /**
    *   @brief  ���Ƽ���
    *
    *   @param  disparity           �Ӳ�ͼ
    *   @param  Q                   ��ͶӰ����
    *
    *   @param  point_cloud
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr calcPointCloud(const cv::Mat& disparity, const cv::Mat& Q) const;

};