#include "projection_picture.h"


void ProjectionPicture::projectHdmi(std::vector<cv::Mat> project_imgs,
									std::vector<cv::Mat>& left_imgs, std::vector<cv::Mat>& right_imgs) {

	for (auto project_img : project_imgs) {
		cv::Mat left_frame, right_frame;
		projectImg(project_img,120);
		_stereo_camera->captureImage(left_frame, right_frame);

		left_imgs.emplace_back(left_frame);
		right_imgs.emplace_back(right_frame);
	}
}

void ProjectionPicture::projectImg(const cv::Mat& img, int wait_time) const {

    cv::imshow("Pattern Window", img);
    cv::waitKey(wait_time);
	
}