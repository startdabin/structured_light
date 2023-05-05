#include "dahen_camera.h"
#include <iostream>
#include <opencv2/highgui.hpp>

bool DahenCamera::camInit() {
	bool init_status=true;

	//遍历枚举相机
	GxIAPICPP::gxdeviceinfo_vector vectorDeviceInfo;
	IGXFactory::GetInstance().UpdateDeviceList(10, vectorDeviceInfo);
	/*for (int i = 0; i < vectorDeviceInfo.size(); i++){
		std::cout << vectorDeviceInfo[i].GetVendorName() << std::endl;
		std::cout << vectorDeviceInfo[i].GetModelName() << std::endl;
		std::cout << vectorDeviceInfo[i].GetSN() << std::endl;
		std::cout << camera_sn << std::endl;
		std::cout << std::endl;
	}*/

	// 打开相机
	_objDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(_camera_sn, GX_ACCESS_EXCLUSIVE);

	// 触发模式设置
	_objFeatureControlPtr = _objDevicePtr->GetRemoteFeatureControl();
	_objFeatureControlPtr->GetEnumFeature("TriggerMode")->SetValue("On");
	// 相机图像尺寸
	auto width = _objFeatureControlPtr->GetIntFeature("Width")->GetMax();
	auto height = _objFeatureControlPtr->GetIntFeature("Height")->GetMax();
	_image_size = { (int)width ,(int)height };
	// 注册回调
	_pCaptureEventHandler = new CSampleCaptureEventHandler();
	_objStreamPtr = _objDevicePtr->OpenStream(0);
	_objStreamPtr->RegisterCaptureCallback(_pCaptureEventHandler, (void*)&_camera_frame);
	_objStreamPtr->StartGrab();

	// 打开采集
	_objFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();

	// 图片buffer初始化
	_camera_frame.frame.create(_image_size, CV_8UC1);
	std::cout << "camera "<< _camera_sn <<" init success!!!" << std::endl;

	return init_status;
}

bool DahenCamera::setExposure(int exposure) {
	// 参数设置
	_objFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(exposure);

	return true;
}

cv::Mat DahenCamera::getFrame(){
	cv::Mat frame;
	_objFeatureControlPtr->GetCommandFeature("TriggerSoftware")->Execute();
	while (true) {
		if (_camera_frame.is_capture_finish) {
			break;
		}
	}
	_camera_frame.is_capture_finish = false;
	_camera_frame.frame.copyTo(frame);
	return frame;
}

bool DahenCamera::close() {
	bool close_status = true;
	//停采、注销采集回调函数
	_objStreamPtr->StopGrab();
	_objStreamPtr->UnregisterCaptureCallback();
	//
	delete _pCaptureEventHandler;
	_pCaptureEventHandler = NULL;
	//关闭流通道
	_objStreamPtr->Close();

	std::cout << "camera " << _camera_sn << " close success!!!" << std::endl;
	return close_status;
}