#include "reconstruction.h"
#include <iostream>
#include <math.h>
#include <opencv2/highgui.hpp>
#include <pcl/io/ply_io.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// 计算视差图
cv::Mat Reconstruction::calcDisparityMap(const cv::Mat& left_code_img, const cv::Mat& right_code_img)const
{
    //_codeImgL = 2*CV_PI;
    auto img_size = left_code_img.size();
    float code_thr = 32 * 2 * CV_PI / img_size.width;
    std::cout << "codeThr: " << code_thr << std::endl;
    cv::Mat disparity_map = cv::Mat::zeros(img_size, CV_32F);

    const float* left_row;
    const float* right_row;

    auto diaparity_data = (float*)disparity_map.data;
    int flag = 0;
    float err = 0;
    for (int i = 0; i < img_size.height; ++i)
    {
        flag = 0;
        left_row = (float*)left_code_img.ptr(i);
        int flag = 0;
        for (int j = 0; j < img_size.width; ++j)
        {   
            right_row = (float*)right_code_img.ptr(i);
            // j - 60  为加速匹配
            
            for (int k = 0; k < img_size.width; ++k)

            {
                if (*left_row < CV_PI)
                    continue;
                err = *right_row - *left_row;
                right_row++;
              
                if (abs(err) < code_thr)
                {
                    flag = k;
                    *diaparity_data = j - k;
                    //cout<<disData[j]<<endl;
                    break;
                }
            }
            left_row++;
            diaparity_data++;
        }
    }
    //cv::imwrite("disparity.jpg", disparity_map);

    //cv::normalize(disparity_map, disparity_map, 0, 1, cv::NORM_MINMAX);
    //cv::imshow("dis_map", disparity_map);
    //cv::waitKey();
    return disparity_map;
}

// 图像极线矫正
cv::Mat Reconstruction::rectifyImg( const cv::Mat& img,
                                    const cv::Mat& camera_matrix,const cv::Mat& distcoeffs,
                                    const cv::Mat& r, const cv::Mat& p) const {

    auto img_size = img.size();
    cv::Mat rmap_first[2], rmap_second[2];
    cv::initUndistortRectifyMap(camera_matrix, distcoeffs, r, p,
        img_size, CV_16SC2, rmap_first[0], rmap_first[1]);

    cv::Mat rectified_img;
    remap(img, rectified_img, rmap_first[0], rmap_first[1], cv::INTER_LINEAR);

    return rectified_img;
}
// 计算点云
pcl::PointCloud<pcl::PointXYZ>::Ptr Reconstruction::calcPointCloud(const cv::Mat& disparity, const cv::Mat& Q) const {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto img_size = disparity.size();
    Eigen::Matrix4d eigenQ;
    cv::cv2eigen(Q, eigenQ);
    float X_temp, Y_temp, Z_temp, W_temp;
    
    auto disparity_data = (float*)disparity.data;
    for (int i = 0; i < img_size.height; i++)
    {
        for (int j = 0; j < img_size.width; j++)
        {
            //pcl::PointXYZ point_temp;
      
            if (*disparity_data == 0) {
                disparity_data++;
                continue;
            }
                
            X_temp = j + eigenQ(0, 3);
            Y_temp = i + eigenQ(1, 3);
            Z_temp = eigenQ(2, 3);
            W_temp = 1.0f / (eigenQ(3, 2) * (*disparity_data) + eigenQ(3, 3));

            point_cloud->push_back({ X_temp * W_temp, Y_temp * W_temp, Z_temp * W_temp});
            disparity_data++;
        }
    }
    return point_cloud;
}

