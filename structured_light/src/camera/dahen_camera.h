#pragma once
#include "camera.h"
#include "GalaxyIncludes.h"

struct CameraFrame
{
	CameraFrame(bool is_finish) { is_capture_finish = is_finish; }
	volatile bool is_capture_finish;
	cv::Mat frame;
};

class CSampleCaptureEventHandler : public ICaptureEventHandler {
public:
	void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam) {
		auto capture_frame = (CameraFrame*)pUserParam;
		if (objImageDataPointer->GetStatus() == GX_FRAME_STATUS_SUCCESS) {

			std::cout << "ÊÕµ½Ò»Ö¡Í¼Ïñ!" << std::endl;

			std::cout << "ImageInfo: " << objImageDataPointer->GetWidth() << std::endl;

			std::cout << "ImageInfo: " << objImageDataPointer->GetHeight() << std::endl;

			std::cout << "ImageInfo: " << objImageDataPointer->GetPayloadSize() << std::endl;
			std::memcpy(capture_frame->frame.data, objImageDataPointer->GetBuffer(), objImageDataPointer->GetPayloadSize());
		}
		capture_frame->is_capture_finish = true;
	}
};

class DahenCamera : public	Camera {

public:
	DahenCamera() = default;
	DahenCamera(const char* camera_sn) : _camera_sn(camera_sn) {};
	~DahenCamera() = default;

	bool camInit();
	cv::Mat getFrame();
	bool setExposure(int exposure);
	bool close();

	static void libInit() { IGXFactory::GetInstance().Init(); }
	static void libUinit() { IGXFactory::GetInstance().Uninit(); }

private:
	CGXDevicePointer _objDevicePtr;
	CGXStreamPointer _objStreamPtr;
	CGXFeatureControlPointer _objFeatureControlPtr;
	ICaptureEventHandler* _pCaptureEventHandler=nullptr;

	const char* _camera_sn;
	cv::Size _image_size;
	CameraFrame _camera_frame = CameraFrame(false);
};


