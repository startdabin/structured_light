#include <iostream>
#include <opencv2/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include "src/structured_light.h"
#include "src/reconstruction.h"
#include "src/utils.h"
#include "src/projection_picture.h"
#include "src/camera/stereo_camera.h"
#include "src/camera/dahen_camera.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructionPicture(std::string img_path);
pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructionCamera();

int main() {

	std::string imgs_path = "E:\\data\\bin_picking\\reconstruction\\demo0\\";
	auto point_cloud = reconstructionPicture(imgs_path);
	//auto point_cloud = reconstructionCamera();
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_scene(new pcl::PointCloud<pcl::PointXYZ>);
	cloudPassThrough(point_cloud, filter_scene, -400, 1500);
	//cloudPassThrough(point_cloud);

	//if (pcl::io::savePCDFile("D:\\data\\table.pcd",*point_cloud)>0)
 //   {
 //       std::cout<<"finish!"<<std::endl;
 //   }
	displayPointCloud(filter_scene);

	
	return 0;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructionCamera() {

	// load structured pattern
	//std::string pattern_dir = "D:\\data\\3d_reconstruction\\structured_light_pattern\\";
	std::string pattern_dir = "D:\\data\\3d_reconstruction\\structured_light_pattern\\structured_light_pattern\\";
	std::string graycode_pattern_path = pattern_dir+"graycode\\";
	std::string graycodeInv_pattern_path = pattern_dir+"graycode_inv\\";
	std::string phase_pattern_path = pattern_dir+"phase\\";
	auto graycode_pattern = loadImgs(graycode_pattern_path, cv::IMREAD_GRAYSCALE);
	auto graycode_inv_pattern = loadImgs(graycodeInv_pattern_path, cv::IMREAD_GRAYSCALE);
	auto phase_pattern = loadImgs(phase_pattern_path, cv::IMREAD_GRAYSCALE);
	// collect images
	std::vector<cv::Mat> left_graycode_imgs, left_inv_graycode_imgs, left_phase_imgs;
	std::vector<cv::Mat> right_graycode_imgs, right_inv_graycode_imgs, right_phase_imgs;
	DahenCamera::libInit();
	Camera* dh_cam_l = new DahenCamera((char*)"KG0210090307");
	Camera* dh_cam_r = new DahenCamera((char*)"KG0210090308");
	dh_cam_l->camInit();
	dh_cam_r->camInit();
	dh_cam_l->setExposure(6000);
	dh_cam_r->setExposure(6000);
	std::shared_ptr<StereoCamera> stereo_cam = std::make_shared<StereoCamera>(dh_cam_l, dh_cam_r);
	ProjectionPicture project_picture(stereo_cam);
	std::cout << graycode_pattern.size() << std::endl;
	project_picture.projectHdmi(graycode_pattern, left_graycode_imgs, right_graycode_imgs);
	project_picture.projectHdmi(graycode_inv_pattern, left_inv_graycode_imgs, right_inv_graycode_imgs);
	project_picture.projectHdmi(phase_pattern, left_phase_imgs, right_phase_imgs);
	// load camera parameter
	std::string camera_param_path = "D:\\data\\3d_reconstruction\\calibrate\\t0408\\stereo_param.yml";
	//std::string camera_param_path = "D:\\data\\3d_reconstruction\\calibrate\\t0408\\intrinsics.yml";
	//std::string camera_param_path = "D:\\PROJECT\\work\\stereo_calibration\\data\\stereo_imgs0\\stereo_param.xml";
	cv::Mat Q1, R1, T1;
	cv::Mat intr_first, intr_second, discoeffs_first, discoeffs_second, r_first, r_second, p_first, p_second;//校正数据
	cv::FileStorage fs(camera_param_path, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> intr_first;
		fs["D1"] >> discoeffs_first;

		fs["M2"] >> intr_second;
		fs["D2"] >> discoeffs_second;

		fs["R"] >> R1;
		fs["T"] >> T1;
		fs["R1"] >> r_first;
		fs["R2"] >> r_second;
		fs["P1"] >> p_first;
		fs["P2"] >> p_second;
		fs["Q"] >> Q1;

		fs.release();
	}
	auto sl_pattern = StructuredLight(5, 4, { 1280,1024 });
	auto left_binary_imgs = sl_pattern.binaryGraycodeImg(left_graycode_imgs, left_inv_graycode_imgs, 12);
	auto left_code_img = sl_pattern.decode(left_binary_imgs, left_phase_imgs);
	auto right_binary_imgs = sl_pattern.binaryGraycodeImg(right_graycode_imgs, right_inv_graycode_imgs, 12);
	auto right_code_img = sl_pattern.decode(right_binary_imgs, right_phase_imgs);

	writeImgs(left_graycode_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_left\\graycode\\", "graycode_");
	writeImgs(left_inv_graycode_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_left\\graycode_inv\\", "graycode_inv_");
	writeImgs(left_phase_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_left\\phase\\", "phase_");

	writeImgs(right_graycode_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_right\\graycode\\", "graycode_");
	writeImgs(right_inv_graycode_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_right\\graycode_inv\\", "graycode_inv_");
	writeImgs(right_phase_imgs, "D:\\data\\3d_reconstruction\\reconstruction_images\\demo0\\img_right\\phase\\", "phase_");


	auto rec = Reconstruction();
	// stere rectify
	left_code_img = rec.rectifyImg(left_code_img, intr_first, discoeffs_first, r_first, p_first);
	right_code_img = rec.rectifyImg(right_code_img, intr_second, discoeffs_second, r_second, p_second);

	//cv::normalize(left_code_img, left_code_img, 0, 1, cv::NORM_MINMAX);
	//cv::normalize(right_code_img, right_code_img, 0, 1, cv::NORM_MINMAX);

	//cv::imshow("left_code_img", left_code_img);
	//cv::imshow("right_code_img", right_code_img);
	//cv::waitKey();
	// calculate disparity
	auto disparity_map = rec.calcDisparityMap(left_code_img, right_code_img);

	// calculate point cloud
	//*point_cloud = *rec.calcPointCloud(disparity_map, Q1);
	return rec.calcPointCloud(disparity_map, Q1);




	pcl::PointCloud<pcl::PointXYZ>::Ptr cc(new pcl::PointCloud<pcl::PointXYZ>);
	// close camera
	dh_cam_l->close();
	dh_cam_r->close();
	delete dh_cam_l;
	delete dh_cam_r;
	dh_cam_l = nullptr;
	dh_cam_r = nullptr;
	DahenCamera::libUinit();

	return cc;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr reconstructionPicture(std::string imgs_path) {
	std::string left_graycode_path = imgs_path + "img_left\\graycode";
	std::string left_inv_graycode_path = imgs_path + "img_left\\graycode_inv";
	std::string left_phase_path = imgs_path + "img_left\\phase";
	auto left_graycode_imgs = loadImgs(left_graycode_path, cv::IMREAD_GRAYSCALE);
	auto left_inv_graycode_imgs = loadImgs(left_inv_graycode_path, cv::IMREAD_GRAYSCALE);
	auto left_phase_imgs = loadImgs(left_phase_path, cv::IMREAD_GRAYSCALE);

	std::string right_graycode_path = imgs_path + "img_right\\graycode";
	std::string right_inv_graycode_path = imgs_path + "img_right\\graycode_inv";
	std::string right_phase_path = imgs_path + "img_right\\phase";
	auto right_graycode_imgs = loadImgs(right_graycode_path, cv::IMREAD_GRAYSCALE);
	auto right_inv_graycode_imgs = loadImgs(right_inv_graycode_path, cv::IMREAD_GRAYSCALE);
	auto right_phase_imgs = loadImgs(right_phase_path, cv::IMREAD_GRAYSCALE);

	auto sl_pattern = StructuredLight(5, 4, { 1280,1024 });
	auto rec = Reconstruction();
	//sl_pattern.createPhaseImg(phase_path);
	//sl_pattern.creatGraycodeImg(graycode_path);

	// load camera parameter
	std::string camera_param_path = imgs_path + "intrinsics.yml";
	//std::string camera_param_path = "D:\\data\\3d_reconstruction\\calibrate\\t0408\\stereo_param.yml";
	cv::Mat Q1, R1, T1;
	cv::Mat intr_first, intr_second, discoeffs_first, discoeffs_second, r_first, r_second, p_first, p_second;//校正数据
	cv::FileStorage fs(camera_param_path, cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> intr_first;
		fs["D1"] >> discoeffs_first;

		fs["M2"] >> intr_second;
		fs["D2"] >> discoeffs_second;

		fs["R"] >> R1;
		fs["T"] >> T1;
		fs["R1"] >> r_first;
		fs["R2"] >> r_second;
		fs["P1"] >> p_first;
		fs["P2"] >> p_second;
		fs["Q"] >> Q1;

		fs.release();
	}

	auto left_binary_imgs = sl_pattern.binaryGraycodeImg(left_graycode_imgs, left_inv_graycode_imgs, 3);
	auto left_code_img = sl_pattern.decode(left_binary_imgs, left_phase_imgs);

	auto right_binary_imgs = sl_pattern.binaryGraycodeImg(right_graycode_imgs, right_inv_graycode_imgs, 3);
	auto right_code_img = sl_pattern.decode(right_binary_imgs, right_phase_imgs);

	// stere rectify
	left_code_img = rec.rectifyImg(left_code_img, intr_first, discoeffs_first, r_first, p_first);
	right_code_img = rec.rectifyImg(right_code_img, intr_second, discoeffs_second, r_second, p_second);

	//cv::normalize(left_code_img, left_code_img, 0, 255, cv::NORM_MINMAX, CV_8U);
	//cv::normalize(right_code_img, right_code_img, 0, 255, cv::NORM_MINMAX, CV_8U);

	//cv::imshow("left_code_img", left_code_img);
	//cv::imshow("right_code_img", right_code_img);
	//cv::waitKey();
	// calculate disparity
	auto disparity_map = rec.calcDisparityMap(left_code_img, right_code_img);

	// calculate point cloud
	//*point_cloud = *rec.calcPointCloud(disparity_map, Q1);
	return rec.calcPointCloud(disparity_map, Q1);

}

