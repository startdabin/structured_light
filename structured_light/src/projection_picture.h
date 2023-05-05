#pragma once
#include "camera/stereo_camera.h"

#include <iostream>
#include<opencv2/core.hpp>
#include <opencv2/highgui.hpp>


class ProjectionPicture
{
public:
	ProjectionPicture()=default;
	ProjectionPicture(std::shared_ptr<StereoCamera> stereo_camera) : _stereo_camera(stereo_camera){
		cv::namedWindow("Pattern Window", cv::WINDOW_NORMAL);
		cv::moveWindow("Pattern Window", 1920, 1080);
		cv::setWindowProperty("Pattern Window", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
	}
	~ProjectionPicture()=default;

	// project picture with hdmi
	void projectHdmi(std::vector<cv::Mat> project_imgs, std::vector<cv::Mat>& left_imgs, std::vector<cv::Mat>& right_imgs);


private:
	void projectImg(const cv::Mat& img, int wait_time = 1000) const;
	std::shared_ptr<StereoCamera> _stereo_camera;
};

