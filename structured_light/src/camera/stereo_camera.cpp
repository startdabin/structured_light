#include "stereo_camera.h"
#include <iostream>
#include <opencv2/highgui.hpp>

void stereo_display(const cv::Mat& left, const cv::Mat& right, int h=512) {
	std::vector<cv::Mat> stereo;
	stereo.emplace_back(left);
	stereo.emplace_back(right);
	cv::Mat show_img;
	cv::hconcat(stereo, show_img);
	cv::namedWindow("Display window", cv::WINDOW_NORMAL);
	int w = int((float)h / show_img.rows * show_img.cols);
	cv::resizeWindow("Display window", w, h);

	cv::imshow("Display window", show_img);
}

void StereoCamera::captureImage(std::string left_path, std::string right_path) {
	std::cout << "press a to capture image!" << std::endl;
	std::cout << "press s to save image!" << std::endl;
	std::cout << "press q to exit!" << std::endl;
	std::cout << std::endl;

	int save_num = 0;
	std::string left_name, right_name;
	
	auto press_menu = 'k';
	cv::Mat left_frame, right_frame;
	left_frame.create(1280, 1024, CV_8UC1);
	right_frame.create(1280, 1024, CV_8UC1);
	while (true)
	{
		stereo_display(left_frame, right_frame);
		press_menu = cv::waitKey();
		if (press_menu == 'a') {
			left_frame = cam_l->getFrame();
			right_frame = cam_r->getFrame();
		
			std::cout << "captured!" << std::endl;
		}
		else if (press_menu == 's') {
			left_name = left_path + "/left_" + std::to_string(save_num) + ".png";
			right_name = right_path + "/right_" + std::to_string(save_num) + ".png";
			save_num++;
			cv::imwrite(left_name,left_frame);
			cv::imwrite(right_name, right_frame);
			std::cout << left_name << std::endl;
			std::cout << right_name << std::endl;
			std::cout << "image saved!!!" << std::endl;
		}
		else if (press_menu == 'q') {
			break;
		}
	}
}

void StereoCamera::captureImage(cv::Mat& left_img, cv::Mat& right_img) {
	cam_l->getFrame().copyTo(left_img);
	cam_r->getFrame().copyTo(right_img);
	//cv::imshow("ss", left_img);
	//cv::waitKey();
}