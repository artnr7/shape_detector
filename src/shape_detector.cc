#include "shape_detector.hpp"

#include <opencv2/core/hal/interface.h>

#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

int main(int argc, char **argv) {
  // Main goal value
  cv::Mat sign_detect_res_img;
  // Primitives
  cv::Mat img;
  sd::ImgReadFirstFile((const char **)argv, img, cv::IMREAD_COLOR);
  cv::Mat blur_img;
  cv::GaussianBlur(img, blur_img, cv::Size(3, 3), 0);
  cv::Mat edges_img;
  cv::Canny(blur_img, edges_img, 100, 200, 3, false);
  cv::Mat hsv_img;
  cv::cvtColor(blur_img, hsv_img, cv::COLOR_BGR2HSV);

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
  static int j = 0;
  int i = 1;
  while (true) {
    sd::ChannelsInRange(channels, channels_ranged, h_min, h_max, s_min, s_max,
                        v_min, v_max);

    cv::Mat hsv_chnls_sum_img;
    sd::ChannelsSum(hsv_chnls_sum_img, channels_ranged);

    cv::Mat hsv_chnls_and_edges_sum_img;
    cv::bitwise_and(hsv_chnls_sum_img, edges_img, hsv_chnls_and_edges_sum_img);

    cv::Mat black_img;
    cv::cvtColor(img, black_img, cv::COLOR_BGR2GRAY);
    black_img.setTo(cv::Scalar(0, 0, 0));

    cv::Mat mask_img;

    sd::FindAndDrawContours(hsv_chnls_sum_img, black_img, mask_img);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(mask_img, mask_img, cv::COLOR_GRAY2BGR);

    img.copyTo(sign_detect_res_img);

    cv::bitwise_and(sign_detect_res_img, mask_img, sign_detect_res_img);

    cv::findContours(hsv_chnls_sum_img, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Mat final = cv::Mat::zeros(img.size(), img.type());

    for (const auto &el : contours) {
      cv::Rect bbox = cv::boundingRect(el);

      // Копируем область из исходного изображения
      cv::Mat object_roi = img(bbox).clone();

      if (!j) {
        cv::imwrite("output/" + std::to_string(i++) + ".jpg", object_roi);
      }

      // Вставляем объект в финальное изображение
      object_roi.copyTo(final(bbox));
    }

    for (const auto &el : contours) {
      cv::Rect bbox = cv::boundingRect(el);

      // Копируем область из исходного изображения
      cv::Mat object_roi = sign_detect_res_img(bbox).clone();

      if (!j) {
        cv::imwrite("output/" + std::to_string(i++) + ".jpg", object_roi);
      }

      // Вставляем объект в финальное изображение
      object_roi.copyTo(final(bbox));
    }

    sd::ShowImages(main_win, final, h_win, s_win, v_win, channels_ranged,
                   hsv_chnls_sum_win, hsv_chnls_sum_img, edges_win, edges_img,
                   hsv_chnls_and_edges_sum_win, hsv_chnls_and_edges_sum_img,
                   mask_win, mask_img);

    // Unfortunatelly I shall move windows after their show proccess(evry time)

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
