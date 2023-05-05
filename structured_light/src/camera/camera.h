#pragma once
#include <opencv2/core.hpp>

class Camera
{
public:
	Camera()=default;
	~Camera()=default;

	virtual bool camInit() = 0;
	virtual bool setExposure(int exposure) = 0;
	virtual cv::Mat getFrame() = 0;
	virtual bool close() = 0;
	
};
