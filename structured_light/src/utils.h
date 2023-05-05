#pragma once
#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>


/**
*	@brief	����Ŀ���ļ�������ͼƬ
*	@param	img_dir	Ŀ���ļ���
*	@param	imread_mode	ͼ���ȡģʽ
*	
*	@return	imgs
*/
std::vector<cv::Mat> loadImgs(std::string imgs_dir, cv::ImreadModes imread_mode);

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadPointClouds(std::string clouds_dir);

void writeImgs(const std::vector<cv::Mat>& imgs, std::string imgs_dir, std::string suffix);


// ��ʱ��
class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {}

    void reset() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    double elapsed() const {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
        return elapsed.count() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};


void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


void cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,int high = 0, int low = 1000);

// pcl point cloud to cv point cloud
cv::Mat fromPCL(const pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud);

// ������Ʒ�����
pcl::PointCloud<pcl::PointNormal>::Ptr calcPointCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int radius = 23);
