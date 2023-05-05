#pragma once
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>
/**
 * @brief The structured light class
 * 格雷玛、相移码解码
 * 生成绝对相位
 */
class StructuredLight
{
public:
    StructuredLight()=default;
    StructuredLight(int graycode_img_num,int phase_img_num,cv::Size img_size) : _graycode_img_nums(graycode_img_num),
                                                                                _phase_img_nums(phase_img_num),
                                                                                _img_size(img_size){}
    ~StructuredLight() = default;

public:
    /**
      *  @brief     格雷码图案生成
      *  @param     save_path   格雷码图案保存路径

      *  @Sample usage:     
     */
    void creatGraycodeImg(const std::string save_path) const;

    /**
      *  @brief     相移码图案生成
      *  @param     save_path   相移码图案保存路径

      *  @Sample usage:
     */
    void createPhaseImg(const std::string save_path) const;

    /**
      *  @brief     格雷码图案解码
      *  @param     graycode_imgs   格雷码图案序列

      *  @Sample usage:
     */
    cv::Mat decodeGraycodeImg(const std::vector<cv::Mat>& graycode_imgs) const;

    /**
      *  @brief     相移码图案解码
      *  @param     phase_imgs   相移码图案序列

      *  @Sample usage:
     */
    cv::Mat decodePhaseImg(const std::vector<cv::Mat>& phase_imgs) const;

    /**
      *  @brief     结构光（格雷码+相移码）图案解码
      * 
      *  @param     graycode_imgs   格雷码图案序列
      *  @param     phase_imgs   相移码图案序列
      * 
     */
    cv::Mat decode(const std::vector<cv::Mat>& graycode_imgs, const std::vector<cv::Mat>& phase_imgs) const;

public:
    /**
      *  @brief     格雷码正反差值二值化
      *
      *  @param     graycode_imgs       格雷码图案序列
      *  @param     inv_graycode_imgs   反向格雷码图案序列
      *  @param     threshold           差值二值化阈值
      *
     */
    std::vector<cv::Mat> binaryGraycodeImg( const std::vector<cv::Mat>& graycode_imgs,
                                            const std::vector<cv::Mat>& inv_graycode_imgs,
                                            int threshold=5);

private:
    int _graycode_img_nums;
    int _phase_img_nums;
    cv::Size _img_size;
};
