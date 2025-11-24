#include "shape_detector.hpp"

#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  // Main goal value
  cv::Mat sign_detect_res_img;
  // Primitives
  // src
  cv::Mat img;
  sd::ImgReadFirstFile((const char **)argv, img, cv::IMREAD_COLOR);
  // blured
  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);
  // edges
  cv::Mat edges_img;
  cv::Canny(blur_img, edges_img, 100, 200, 3, false);
  // hsv channels img
  cv::Mat hsv_img;
  cv::cvtColor(blur_img, hsv_img, cv::COLOR_BGR2HSV);
  // black
  cv::Mat black_img;
  cv::cvtColor(img, black_img, cv::COLOR_BGR2GRAY);
  black_img.setTo(cv::Scalar(0, 0, 0));
  // mask
  cv::Mat mask_img;
  img.copyTo(mask_img);
  // colorized mask
  cv::Mat clr_mask_img;
  img.copyTo(clr_mask_img);

  // Windows
  std::string main_win{"Rawd Sign Detector"};
  std::string h_win{"h channel"};
  std::string s_win{"s channel"};
  std::string v_win{"v channel"};
  std::string hsv_chnls_sum_win{"Channels sum"};
  std::string edges_win{"Canny edges"};
  std::string hsv_chnls_and_edges_sum_win{"Edges and channels"};
  std::string mask_win{"Mask"};

  sd::MakeWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                  hsv_chnls_and_edges_sum_win, mask_win);

  sd::ResizeWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                    hsv_chnls_and_edges_sum_win, mask_win);
  // Windows----------------------------------------------------------------------

  // HSV Channels
  std::vector<cv::Mat> channels;
  cv::split(hsv_img, channels);

  // HSV channels trackbars
  int h_min = 0, h_max = 0;
  int s_min = 0, s_max = 0;
  int v_min = 0, v_max = 0;

  sd::ChangeTrackbarsValues(h_min, h_max, s_min, s_max, v_min, v_max);
  sd::CreateTrackbars(channels, h_win, s_win, v_win, h_min, h_max, s_min, s_max,
                      v_min, v_max);

  // Create InRange Channels
  std::vector<cv::Mat> channels_ranged;
  sd::ChannelsRangedCreate(channels, channels_ranged);

  // Main Cycle
  int j = 0;
  int output_imgs_cnt = 1;

  int clr_mode = 0;
  while (true) {
    clr_mode %= COLOR_QTY;
    ++clr_mode;
    // sd::SetColorMode(clr_mode, h_min, h_max);`
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_chnls_sum_img;
    sd::ChannelsSum(hsv_chnls_sum_img, channels_ranged);

    cv::Mat hsv_chnls_and_edges_sum_img;
    cv::bitwise_and(hsv_chnls_sum_img, edges_img, hsv_chnls_and_edges_sum_img);

    sd::FindAndFillContours(hsv_chnls_sum_img, black_img, mask_img);

    sd::SetColorizedMask(img, mask_img, clr_mask_img);

    cv::Mat sign_detect_res_img = cv::Mat::zeros(img.size(), img.type());

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(hsv_chnls_sum_img, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // // Вырезаем из исходного изображения, получаются прямоугольники где
    // объект
    // // на фоне всего
    // sd::WriteContoursRect(contours, img, output_imgs_cnt, j,
    //                       sign_detect_res_img);
    // Вырезаем из исходного изображения, получаются прямоугольники где объект
    // на чёрном фоне
    sd::WriteContoursRect(contours, clr_mask_img, output_imgs_cnt, j,
                          sign_detect_res_img);

    // for (const auto &el : contours) {
    //   double area = cv::contourArea(el);
    //   double perimeter = cv::arcLength(el, true);

    //   std::vector<cv::Point> approx;
    //   cv::approxPolyDP(el, approx, 0.02 * perimeter, true);

    //   if (approx.size() == 4 && area > 10) {
    //     cv::drawContours(img, std::vector<std::vector<cv::Point>>{el}, -1,
    //                      cv::Scalar(0, 255, 0), 3);
    //   }
    // }

    for (const auto &contour : contours) {
      double area = cv::contourArea(contour);
      double perimeter = cv::arcLength(contour, true);

      // circularity
      double circularity = 4 * CV_PI * (area / (perimeter * perimeter));
      if (circularity < 0.6) continue;  // порог на круговую форму

      if (contour.size() >= 5) {
        cv::RotatedRect ellipse = cv::fitEllipse(contour);
        double ratio = ellipse.size.width / ellipse.size.height;
        if (ratio < 0.6 || ratio > 1.4)
          continue;  
      }

      // Если прошли фильтры — контур подходит как круг
      // Можно нарисовать контур
      cv::drawContours(img, std::vector<std::vector<cv::Point>>{contour}, -1,
                       cv::Scalar(0, 255, 0), 3);
    }

    sd::ShowImages(main_win, sign_detect_res_img, h_win, s_win, v_win,
                   channels_ranged, hsv_chnls_sum_win, hsv_chnls_sum_img,
                   edges_win, edges_img, hsv_chnls_and_edges_sum_win,
                   hsv_chnls_and_edges_sum_img, mask_win, mask_img);

    // Unfortunatelly I shall move windows after their show proccess(every time)
    sd::MoveWindows(main_win, h_win, s_win, v_win, hsv_chnls_sum_win, edges_win,
                    hsv_chnls_and_edges_sum_win, mask_win);

    // App exit
    if (cv::waitKey(PAUSE) == 27) {
      break;
    }
    ++j;
  }

  cv::destroyAllWindows();
  (void)argc;
  return 0;
}
