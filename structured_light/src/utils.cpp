#include "utils.h"

#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>


std::vector<cv::Mat> loadImgs(std::string imgs_dir, cv::ImreadModes imread_mode) {
    std::vector<cv::Mat> imgs;
    std::vector<std::string> imgs_path;
    cv::glob(imgs_dir, imgs_path);
    for (auto img_path : imgs_path) {
        imgs.push_back(cv::imread(img_path, imread_mode));
    }
    return imgs;
}
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> loadPointClouds(std::string clouds_dir) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    std::vector<std::string> clouds_path;
    cv::glob(clouds_dir, clouds_path);
    for (auto cloud_path : clouds_path) {
        auto cloud = new pcl::PointCloud<pcl::PointXYZ>();
        pcl::io::loadPCDFile(cloud_path, *cloud);
        clouds.emplace_back(cloud);
    }
    return clouds;
}

void writeImgs(const std::vector<cv::Mat>& imgs, std::string imgs_dir, std::string suffix) {
    for (int i = 0; i < imgs.size(); i++) {
        auto img_name = imgs_dir + suffix + std::to_string(i) + ".png";
        cv::imwrite(img_name, imgs[i]);
    }
}


void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Create a PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->addCoordinateSystem(50);
    // Set the background color to black
    viewer->setBackgroundColor(0, 0, 0);

    // Add the point cloud to the viewer
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    // Set the point size to 1
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    // Enable the keyboard event handler
    viewer->registerKeyboardCallback([](const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
        if (event.getKeySym() == "q" && event.keyDown()) {
            static_cast<pcl::visualization::PCLVisualizer*>(viewer_void)->close();
        }
        }, (void*)viewer.get());

    // Display the point cloud until the user quits
    viewer->spin();
}


void cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, int high , int low) {

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(high, low);

    pass.filter(*output_cloud);

}
// pcl point cloud to cv point cloud
cv::Mat fromPCL(const pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud) {
    cv::Mat pc_mat = cv::Mat(point_cloud->size(), 6, CV_32FC1);
    int j = 0;
    for (auto i = point_cloud->begin(); i != point_cloud->end(); i++)
    {
        float* data = pc_mat.ptr<float>(j);
        int col = 0;
        data[0] = i->x;
        data[1] = i->y;
        data[2] = i->z;
        data[3] = i->normal_x;
        data[4] = i->normal_y;
        data[5] = i->normal_z;
        ++j;
        // normalize to unit norm
        double norm = sqrt(data[3] * data[3] + data[4] * data[4] + data[5] * data[5]);
        if (norm > 0.00001)
        {
            data[3] /= static_cast<float>(norm);
            data[4] /= static_cast<float>(norm);
            data[5] /= static_cast<float>(norm);
        }

    }
    return pc_mat;
}

// 计算点云法向量
pcl::PointCloud<pcl::PointNormal>::Ptr calcPointCloudNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int radius) {
    pcl::PointCloud<pcl::PointNormal>::Ptr result = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::PointCloud<pcl::Normal>::Ptr normal = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    // compute normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch(radius);
    norm_est.setInputCloud(cloud);
    norm_est.compute(*normal);

    // concat cloud with normals
    pcl::concatenateFields(*cloud, *normal, *result);

    return result;
}
//auto imgs = loadImgs("D:\\data\\3d_reconstruction\\structured_light_pattern\\graycode\\", cv::IMREAD_GRAYSCALE);
//for (int i = 0; i < imgs.size(); i++) {
//    auto save_name = "D:\\data\\3d_reconstruction\\structured_light_pattern\\graycode_inv\\graycode_inv_" + std::to_string(i) + ".bmp";
//    auto inv_img = 255 - imgs[i];
//    cv::imwrite(save_name, inv_img);
//}
