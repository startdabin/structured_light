#include "structured_light.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


/**
  *  @brief     格雷码图案生成
  *  @param     save_path   格雷码图案保存路径

  *  @Sample usage:
 */
void StructuredLight::creatGraycodeImg(const std::string save_path) const{
    for (int i = 0; i < _graycode_img_nums; i++)
    {
        cv::Mat m = cv::Mat::zeros(_img_size, CV_8U);
        int stripe_width = int(_img_size.width / pow(2, i));
        cv::Mat stripe(_img_size.height, stripe_width, CV_8U, 255);

        int bright_position = stripe_width / 2;
        if (i == 0)
        {
            cv::Mat stripe(_img_size.height, bright_position, CV_8U, 255);
            stripe.copyTo(m.colRange(bright_position, stripe_width));
        }
        else
        {
            for (int j = 0; j < pow(2, i); j += 2)
            {
                stripe.copyTo(m.colRange(bright_position + j * stripe_width, bright_position + (j + 1) * stripe_width));
            }
        }

        std::string imgname = save_path + "/grayimg_" + std::to_string(i) + ".bmp";
        cv::imwrite(imgname, m);
        //gray_img.push_back(m);
    }
    std::cout << "graycode image saved in:  " << save_path << std::endl;

}

/**
  *  @brief     相移码图案生成
  *  @param     save_path   相移码图案保存路径

  *  @Sample usage:
 */
void StructuredLight::createPhaseImg(const std::string save_path) const{

    cv::Mat m = cv::Mat::zeros(_img_size, CV_8U);
    int stripe_width = int(_img_size.width / pow(2, _graycode_img_nums));
    cv::Mat stripe(_img_size.height, stripe_width, CV_8U, 255);//条纹
    float stride = 2 * CV_PI / stripe_width;
    float half_pi = CV_PI * 0.5f;
    for (int i = 0; i < _phase_img_nums; i++)
    {
        
        for (int j = 0; j < stripe.cols; j++){
            auto theta = stride * j; //将一个周期的像素坐标映射到0-2pi

            //计算每个像素对应的余弦值，范围是0-255（8位），也可用更高位的图像
            int piv = int(128 + 127 * cos(theta + half_pi * i));
            cv::Mat pix(_img_size.height, 1, CV_8U, piv);
            pix.copyTo(stripe.col(j));
        }
        for (int k = 0; k < pow(2, _graycode_img_nums); k++)
        {
            stripe.copyTo(m.colRange(k * stripe_width, (k + 1) * stripe_width));
        }
        std::string imgname = save_path + "/phaseimg_" + std::to_string(i) + ".bmp";
        cv::imwrite(imgname, m);
        //gray_img.push_back(m);
    }
    std::cout << "phase image saved in:  " << save_path << std::endl;
}

/**
      *  @brief     格雷码图案解码
      *  @param     graycode_imgs   格雷码图案序列

  *  @Sample usage:
 */
cv::Mat StructuredLight::decodeGraycodeImg(const std::vector<cv::Mat>& graycode_imgs) const {
    auto img_size = graycode_imgs[0].size();
    cv::Mat decode_img(img_size, CV_32F);
    int graycode = 0;
    float phase = 0;
    float code_value = 0, code_value_back = 0;

    int pix = 0;
    
    auto decode_img_data = (float*)decode_img.data;
    std::vector<uchar*> imgs_data;
    for (size_t i = 0; i < _graycode_img_nums; i++) {

        imgs_data.push_back(graycode_imgs[i].data);
    }
    for (int h = 0; h < img_size.height; ++h)
    {
        for (int w = 0; w < img_size.width; ++w)
        {
            pix = 0;
            for (int i = 0; i < _graycode_img_nums;i++) {
                if (*imgs_data[i] > 240) {
                    pix = pix | (1 << (_graycode_img_nums - i - 1));
                }
                //std::cout << pix << std::endl;
                imgs_data[i]++;
            }

            for (int i = 0; i < _graycode_img_nums; i++) {
                if (i == 0)
                {
                    graycode = (pix >> (_graycode_img_nums - 1) & 1) * pow(2, _graycode_img_nums - 1);
                }
                else
                {
                    graycode = graycode + ((pix >> (_graycode_img_nums - i - 1) & 1) ^
                        (graycode >> (_graycode_img_nums - i + 1 - 1) & 1)) *
                        pow(2, _graycode_img_nums - 1 - i);
                }
            }

            //std::cout<<graycode<<std::endl;
            code_value_back = code_value;
            code_value = 2 * graycode * CV_PI;
            // 消除周期边缘误差
            //if (codeValue - codeValueBack >= CV_PI)
            //    codeValue -= 2 * CV_PI;
            //cout<<codeValue - codeValueBack<<endl;
            *decode_img_data = code_value;
            decode_img_data++;
        }
    }
    return decode_img;
}

/**
  *  @brief     相移码图案解码
  *  @param     phase_imgs   相移码图案序列

  *  @Sample usage:
 */
cv::Mat StructuredLight::decodePhaseImg(const std::vector<cv::Mat>& phase_imgs) const {
    auto img_size = phase_imgs[0].size();
    cv::Mat decode_img(img_size, CV_32F);

    auto decode_img_data = (float*)decode_img.data;
    std::vector<uchar*> imgs_data;
    for (size_t i = 0; i < _phase_img_nums; i++) {

        imgs_data.push_back(phase_imgs[i].data);
    }
    float code_value = 0;
    for (int h = 0; h < img_size.height; ++h){
        for (int w = 0; w < img_size.width; ++w){
            //解相位并映射到0-2pi
            code_value = atan2(*imgs_data[1] - *imgs_data[3],
                *imgs_data[2] - *imgs_data[0]);//theta(-pi~pi)
            code_value += CV_PI;

            *decode_img_data = code_value;
            decode_img_data++;

            imgs_data[0]++;
            imgs_data[1]++;
            imgs_data[2]++;
            imgs_data[3]++;
        }
    }
    return decode_img;
}

/**
      *  @brief     结构光（格雷码+相移码）图案解码
      *
      *  @param     graycode_imgs   格雷码图案序列
      *  @param     phase_imgs      相移码图案序列
      * 
      *  @return    decode_img      解码图案
     */
cv::Mat StructuredLight::decode(const std::vector<cv::Mat>& graycode_imgs, const std::vector<cv::Mat>& phase_imgs) const{
    auto img_size = graycode_imgs[0].size();
    cv::Mat decode_img(img_size, CV_32F);
    int graycode = 0;
    float phase = 0;
    float code_value = 0, code_value_back = 0;

    int pix = 0;

    auto decode_img_data = (float*)decode_img.data;
    std::vector<uchar*> graycode_imgs_data;
    std::vector<uchar*> phase_imgs_data;
    for (size_t i = 0; i < _graycode_img_nums; i++) {

        graycode_imgs_data.push_back(graycode_imgs[i].data);
    }
    for (size_t i = 0; i < _phase_img_nums; i++) {

        phase_imgs_data.push_back(phase_imgs[i].data);
    }
    for (int h = 0; h < img_size.height; ++h)
    {
        for (int w = 0; w < img_size.width; ++w)
        {
            pix = 0;
            for (int i = 0; i < _graycode_img_nums; i++) {
                if (*graycode_imgs_data[i] > 240) {
                    pix = pix | (1 << (_graycode_img_nums - i - 1));
                }
                graycode_imgs_data[i]++;
            }

            for (int i = 0; i < _graycode_img_nums; i++) {
                if (i == 0)
                {
                    graycode = (pix >> (_graycode_img_nums - 1) & 1) * pow(2, _graycode_img_nums - 1);
                }
                else
                {
                    graycode = graycode + ((pix >> (_graycode_img_nums - i - 1) & 1) ^
                        (graycode >> (_graycode_img_nums - i + 1 - 1) & 1)) *
                        pow(2, _graycode_img_nums - 1 - i);
                }
            }

            //解相位并映射到0-2pi
            phase = atan2(*phase_imgs_data[1] - *phase_imgs_data[3],
                *phase_imgs_data[2] - *phase_imgs_data[0]);//theta(-pi~pi)
            phase += CV_PI;

            phase_imgs_data[0]++;
            phase_imgs_data[1]++;
            phase_imgs_data[2]++;
            phase_imgs_data[3]++;


            code_value_back = code_value;
            code_value = 2 * graycode * CV_PI + phase;
            // 消除周期边缘误差
            if (phase > 0.8f * CV_2PI && (code_value - code_value_back > CV_PI)) {
                code_value -= CV_2PI;
            }
            else if (phase < 0.2f * CV_2PI && (code_value - code_value_back < -CV_PI)) {
                code_value += CV_2PI;
            }


            *decode_img_data = code_value;
            decode_img_data++;
        }
    }
    return decode_img;
}


std::vector<cv::Mat> StructuredLight::binaryGraycodeImg(const std::vector<cv::Mat>& graycode_imgs,
                                                        const std::vector<cv::Mat>& inv_graycode_imgs,
                                                        int threshold) {
    std::vector<cv::Mat> binary_graycode_imgs;
    int total = graycode_imgs[0].total();

    std::vector<uchar*> imgs_data;
    std::vector<uchar*> inv_imgs_data;
    for (int i = 0; i < graycode_imgs.size(); i++) {
        imgs_data.push_back(graycode_imgs[i].data);
        inv_imgs_data.push_back(inv_graycode_imgs[i].data);
    }
    
    for (int i = 0; i < graycode_imgs.size(); i++) {
        cv::Mat binary_img(graycode_imgs[0].size(), CV_8U);
        auto binary_img_data = binary_img.data;
        for (int j = 0; j < total; j++) {
            if (*imgs_data[i] - *inv_imgs_data[i] >= threshold) {
                *binary_img_data = 255;
            }
            else
            {
                *binary_img_data = 0;
            }
            imgs_data[i]++;
            inv_imgs_data[i]++;
            binary_img_data++;

        }
        binary_graycode_imgs.push_back(binary_img);
    }
    return binary_graycode_imgs;
}